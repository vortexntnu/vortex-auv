#include "dp_adapt_backs_controller/dp_adapt_backs_controller_ros.hpp"
#include <iostream>
#include <variant>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

DPAdaptBacksControllerNode::DPAdaptBacksControllerNode()
    : LifecycleNode("dp_adapt_backs_controller_node") {
    set_adap_params();
}

LifecycleCallbackReturn DPAdaptBacksControllerNode::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "configuration step");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    auto best_effort =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    auto reliable =
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();

    Topics topics;
    get_topics(topics);

    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            topics.dp_reference_topic, qos_sensor_data,
            std::bind(&DPAdaptBacksControllerNode::guidance_callback, this,
                      std::placeholders::_1));

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        topics.pose_topic, best_effort,
        std::bind(&DPAdaptBacksControllerNode::pose_callback, this,
                  std::placeholders::_1));

    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        topics.twist_topic, best_effort,
        std::bind(&DPAdaptBacksControllerNode::twist_callback, this,
                  std::placeholders::_1));

    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        topics.software_kill_switch_topic, best_effort,
        std::bind(&DPAdaptBacksControllerNode::killswitch_callback, this,
                  std::placeholders::_1));

    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        topics.software_operation_mode_topic, best_effort,
        std::bind(&DPAdaptBacksControllerNode::software_mode_callback, this,
                  std::placeholders::_1));

    time_step_ = std::chrono::milliseconds(10);
    tau_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        topics.control_topic, reliable);
    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&DPAdaptBacksControllerNode::publish_tau, this));
    tau_pub_timer_->cancel();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DPAdaptBacksControllerNode::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "activation step");
    tau_pub_timer_->reset();
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DPAdaptBacksControllerNode::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "deactivation step");

    tau_pub_timer_->reset();
    tau_pub_timer_->cancel();
    dp_adapt_backs_controller_.reset_adap_param();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DPAdaptBacksControllerNode::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "cleanup step");

    dp_adapt_backs_controller_.reset_adap_param();
    tau_pub_timer_->reset();
    tau_pub_timer_->cancel();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn DPAdaptBacksControllerNode::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "shutdown step");

    dp_adapt_backs_controller_.reset_adap_param();
    tau_pub_timer_->cancel();
    tau_pub_.reset();

    return LifecycleCallbackReturn::SUCCESS;
}

void DPAdaptBacksControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
}

void DPAdaptBacksControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
}

void DPAdaptBacksControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    eta_.pos << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z;

    eta_.ori = quat_to_euler(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

void DPAdaptBacksControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    nu_.linear_speed << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;

    nu_.angular_speed << msg->twist.twist.angular.x, msg->twist.twist.angular.y,
        msg->twist.twist.angular.z;
}

void DPAdaptBacksControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    dp_types::Vector6d tau =
        dp_adapt_backs_controller_.calculate_tau(eta_, eta_d_, nu_);

    geometry_msgs::msg::Wrench tau_msg;
    tau_msg.force.x = tau(0);
    tau_msg.force.y = tau(1);
    tau_msg.force.z = tau(2);
    tau_msg.torque.x = tau(3);
    tau_msg.torque.y = tau(4);
    tau_msg.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void DPAdaptBacksControllerNode::set_adap_params() {
    this->declare_parameter<std::vector<double>>(
        "adap_param",
        {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8});
    this->declare_parameter<std::vector<double>>(
        "d_gain", {0.3, 0.3, 0.45, 0.2, 0.7, 0.6});
    this->declare_parameter<std::vector<double>>(
        "K1", {20.5, 15.5, 20.5, 1.2, 6.0, 2.5});
    this->declare_parameter<std::vector<double>>(
        "K2", {30.5, 25.5, 30.5, 2.6, 10.0, 6.5});
    this->declare_parameter<std::vector<double>>("r_b_bg", {0.01, 0.0, 0.02});
    this->declare_parameter<std::vector<double>>("I_b", {0.68, 3.32, 3.34});
    this->declare_parameter<std::vector<double>>("mass_matrix",
                                                 std::vector<double>(36, 1.0));
    this->declare_parameter<double>("m", {30});

    std::vector<double> adap_param_vec =
        this->get_parameter("adap_param").as_double_array();
    std::vector<double> d_gain_vec =
        this->get_parameter("d_gain").as_double_array();
    std::vector<double> K1_vec = this->get_parameter("K1").as_double_array();
    std::vector<double> K2_vec = this->get_parameter("K2").as_double_array();
    std::vector<double> r_b_bg_vec =
        this->get_parameter("r_b_bg").as_double_array();
    std::vector<double> I_b_vec = this->get_parameter("I_b").as_double_array();
    std::vector<double> mass_matrix_vec =
        this->get_parameter("mass_matrix").as_double_array();

    double m = this->get_parameter("m").as_double();

    dp_types::Vector12d adap_param_eigen =
        Eigen::Map<dp_types::Vector12d>(adap_param_vec.data());
    dp_types::Vector6d d_gain_eigen =
        Eigen::Map<dp_types::Vector6d>(d_gain_vec.data());
    dp_types::Vector6d K1_eigen = Eigen::Map<dp_types::Vector6d>(K1_vec.data());
    dp_types::Vector6d K2_eigen = Eigen::Map<dp_types::Vector6d>(K2_vec.data());
    dp_types::Vector3d r_b_bg_eigen =
        Eigen::Map<dp_types::Vector3d>(r_b_bg_vec.data());
    dp_types::Vector3d I_b_eigen =
        Eigen::Map<dp_types::Vector3d>(I_b_vec.data());
    dp_types::Matrix6d mass_matrix =
        Eigen::Map<dp_types::Matrix6d>(mass_matrix_vec.data());

    dp_adapt_backs_controller_.set_k1(K1_eigen);
    dp_adapt_backs_controller_.set_k2(K2_eigen);
    dp_adapt_backs_controller_.set_rbg(r_b_bg_eigen);
    dp_adapt_backs_controller_.set_adap_param(adap_param_eigen);
    dp_adapt_backs_controller_.set_d_gain(d_gain_eigen);
    dp_adapt_backs_controller_.set_inertia_matrix(I_b_eigen);
    dp_adapt_backs_controller_.set_mass_inertia_matrix(mass_matrix);
    dp_adapt_backs_controller_.set_m(m);
}

void DPAdaptBacksControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    eta_d_.pos << msg->x, msg->y, msg->z;
    eta_d_.ori << msg->roll, msg->pitch, msg->yaw;
}

void DPAdaptBacksControllerNode::get_topics(Topics& topics) {
    this->declare_parameter("dp_reference_topic", "/dp/reference");
    topics.dp_reference_topic =
        this->get_parameter("dp_reference_topic").as_string();

    this->declare_parameter("pose_topic", "/dvl/pose");
    topics.pose_topic = this->get_parameter("pose_topic").as_string();

    this->declare_parameter("twist_topic", "/dvl/twist");
    topics.twist_topic = this->get_parameter("twist_topic").as_string();

    this->declare_parameter("software_kill_switch_topic",
                            "/softwareKillSwitch");
    topics.software_kill_switch_topic =
        this->get_parameter("software_kill_switch_topic").as_string();

    this->declare_parameter("software_operation_mode_topic",
                            "/softwareOperationMode");
    topics.software_operation_mode_topic =
        this->get_parameter("software_operation_mode_topic").as_string();

    this->declare_parameter("control_topic", "/thrust/wrench_input");
    topics.control_topic = this->get_parameter("control_topic").as_string();
}

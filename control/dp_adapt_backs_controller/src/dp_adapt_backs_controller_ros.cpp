#include "dp_adapt_backs_controller/dp_adapt_backs_controller_ros.hpp"
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

DPAdaptBacksControllerNode::DPAdaptBacksControllerNode(
    const rclcpp::NodeOptions& options)
    : Node("dp_adapt_backs_controller_node", options) {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&DPAdaptBacksControllerNode::publish_tau, this));

    set_adap_params();
}

void DPAdaptBacksControllerNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    this->declare_parameter<std::string>("topics.guidance.dp");
    std::string dp_reference_topic =
        this->get_parameter("topics.guidance.dp").as_string();
    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, qos_sensor_data,
            std::bind(&DPAdaptBacksControllerNode::guidance_callback, this,
                      std::placeholders::_1));

    this->declare_parameter<std::string>("topics.pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&DPAdaptBacksControllerNode::pose_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.twist");
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        std::bind(&DPAdaptBacksControllerNode::twist_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.killswitch");
    std::string software_kill_switch_topic =
        this->get_parameter("topics.killswitch").as_string();
    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, 1,
        std::bind(&DPAdaptBacksControllerNode::killswitch_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.operation_mode");
    std::string software_operation_mode_topic =
        this->get_parameter("topics.operation_mode").as_string();
    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        software_operation_mode_topic, 1,
        std::bind(&DPAdaptBacksControllerNode::software_mode_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.wrench_input");
    std::string control_topic =
        this->get_parameter("topics.wrench_input").as_string();
    tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        control_topic, qos_sensor_data);
}

void DPAdaptBacksControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Killswitch: %s",
                killswitch_on_ ? "on" : "off");
}

void DPAdaptBacksControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Software mode: %s",
                software_mode_.c_str());

    if (software_mode_ == "autonomous mode") {
        eta_d_ = eta_;
    }
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

void DPAdaptBacksControllerNode::set_adap_params() {
    this->declare_parameter<std::vector<double>>("adapt_gain");
    this->declare_parameter<std::vector<double>>("d_gain");
    this->declare_parameter<std::vector<double>>("K1");
    this->declare_parameter<std::vector<double>>("K2");
    this->declare_parameter<std::vector<double>>("r_b_bg");
    this->declare_parameter<std::vector<double>>("inertia_matrix");
    this->declare_parameter<std::vector<double>>("mass_matrix");
    this->declare_parameter<double>("m");

    std::vector<double> adap_param_vec =
        this->get_parameter("adapt_gain").as_double_array();
    std::vector<double> d_gain_vec =
        this->get_parameter("d_gain").as_double_array();
    std::vector<double> K1_vec = this->get_parameter("K1").as_double_array();
    std::vector<double> K2_vec = this->get_parameter("K2").as_double_array();
    std::vector<double> r_b_bg_vec =
        this->get_parameter("r_b_bg").as_double_array();
    std::vector<double> I_b_vec =
        this->get_parameter("inertia_matrix").as_double_array();
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

    dp_adapt_params_.adap_param = adap_param_eigen;
    dp_adapt_params_.d_gain = d_gain_eigen;
    dp_adapt_params_.K1 = K1_eigen;
    dp_adapt_params_.K2 = K2_eigen;
    dp_adapt_params_.r_b_bg = r_b_bg_eigen;
    dp_adapt_params_.I_b = I_b_eigen;
    dp_adapt_params_.mass_matrix = mass_matrix;
    dp_adapt_params_.m = m;

    dp_adapt_backs_controller_ =
        std::make_unique<DPAdaptBacksController>(dp_adapt_params_);
    ;
}

void DPAdaptBacksControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    dp_types::Vector6d tau =
        dp_adapt_backs_controller_->calculate_tau(eta_, eta_d_, nu_);

    geometry_msgs::msg::WrenchStamped tau_msg;
    tau_msg.header.stamp = this->now();
    tau_msg.header.frame_id = "base_link";
    tau_msg.wrench.force.x = tau(0);
    tau_msg.wrench.force.y = tau(1);
    tau_msg.wrench.force.z = tau(2);
    tau_msg.wrench.torque.x = tau(3);
    tau_msg.wrench.torque.y = tau(4);
    tau_msg.wrench.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void DPAdaptBacksControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    eta_d_.pos << msg->x, msg->y, msg->z;
    eta_d_.ori << msg->roll, msg->pitch, msg->yaw;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DPAdaptBacksControllerNode)

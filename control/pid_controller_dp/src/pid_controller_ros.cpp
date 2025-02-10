#include <pid_controller_dp/pid_controller_ros.hpp>
#include <variant>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

PIDControllerNode::PIDControllerNode() : Node("pid_controller_node") {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&PIDControllerNode::publish_tau, this));
    set_pid_params();
}

void PIDControllerNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    this->declare_parameter<std::string>("topics.namespace");
    std::string ns = this->get_parameter("topics.namespace").as_string();

    this->declare_parameter<std::string>("topics.guidance.dp");
    std::string dp_reference_topic =
        ns + this->get_parameter("topics.guidance.dp").as_string();

    this->declare_parameter<std::string>("topics.pose");
    std::string pose_topic =
        ns + this->get_parameter("topics.pose").as_string();

    this->declare_parameter<std::string>("topics.twist");
    std::string twist_topic =
        ns + this->get_parameter("topics.twist").as_string();

    this->declare_parameter<std::string>("topics.killswitch");
    std::string software_kill_switch_topic =
        ns + this->get_parameter("topics.killswitch").as_string();

    this->declare_parameter<std::string>("topics.operation_mode");
    std::string software_operation_mode_topic =
        ns + this->get_parameter("topics.operation_mode").as_string();

    this->declare_parameter<std::string>("topics.wrench_input");
    std::string control_topic =
        ns + this->get_parameter("topics.wrench_input").as_string();

    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, 10,
        std::bind(&PIDControllerNode::killswitch_callback, this,
                  std::placeholders::_1));
    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        software_operation_mode_topic, 10,
        std::bind(&PIDControllerNode::software_mode_callback, this,
                  std::placeholders::_1));

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::pose_callback, this,
                  std::placeholders::_1));

    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::twist_callback, this,
                  std::placeholders::_1));

    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, qos_sensor_data,
            std::bind(&PIDControllerNode::guidance_callback, this,
                      std::placeholders::_1));

    tau_pub_ =
        this->create_publisher<geometry_msgs::msg::Wrench>(control_topic, 10);
}

void PIDControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
}

void PIDControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
}

void PIDControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    eta_ = eta_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    nu_ = nu_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    types::Vector6d tau =
        pid_controller_.calculate_tau(eta_, eta_d_, nu_, eta_dot_d_);

    geometry_msgs::msg::Wrench tau_msg;
    tau_msg.force.x = tau(0);
    tau_msg.force.y = tau(1);
    tau_msg.force.z = tau(2);
    tau_msg.torque.x = tau(3);
    tau_msg.torque.y = tau(4);
    tau_msg.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void PIDControllerNode::set_pid_params() {
    this->declare_parameter<std::vector<double>>(
        "Kp", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "Ki", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<std::vector<double>>(
        "Kd", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::vector<double> Kp_vec = this->get_parameter("Kp").as_double_array();
    std::vector<double> Ki_vec = this->get_parameter("Ki").as_double_array();
    std::vector<double> Kd_vec = this->get_parameter("Kd").as_double_array();

    types::Matrix6d Kp_eigen = Eigen::Map<types::Matrix6d>(Kp_vec.data());
    types::Matrix6d Ki_eigen = Eigen::Map<types::Matrix6d>(Ki_vec.data());
    types::Matrix6d Kd_eigen = Eigen::Map<types::Matrix6d>(Kd_vec.data());

    pid_controller_.set_kp(Kp_eigen);
    pid_controller_.set_ki(Ki_eigen);
    pid_controller_.set_kd(Kd_eigen);
}

void PIDControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    eta_d_.pos << msg->x, msg->y, msg->z;

    double roll = msg->roll;
    double pitch = msg->pitch;
    double yaw = msg->yaw;

    eta_d_.ori = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

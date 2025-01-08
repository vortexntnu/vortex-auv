#include <iostream>
#include <pid_controller_dp/pid_controller_ros.hpp>
#include <variant>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

PIDControllerNode::PIDControllerNode() : Node("pid_controller_node") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    time_step_ = std::chrono::milliseconds(10);

    this->declare_parameter("nucleus_odom_topic", "/orca/odom");
    this->declare_parameter("dp_reference_topic", "/dp/reference");
    this->declare_parameter("control_topic", "/thrust/wrench_input");

    std::string nucleus_odom_topic =
        this->get_parameter("nucleus_odom_topic").as_string();
    std::string dp_reference_topic =
        this->get_parameter("dp_reference_topic").as_string();
    std::string control_topic =
        this->get_parameter("control_topic").as_string();

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        nucleus_odom_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::odometry_callback, this,
                  std::placeholders::_1));
    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, qos_sensor_data,
            std::bind(&PIDControllerNode::guidance_callback, this,
                      std::placeholders::_1));
    kp_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid/kp", qos_sensor_data,
        std::bind(&PIDControllerNode::kp_callback, this,
                  std::placeholders::_1));
    ki_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid/ki", qos_sensor_data,
        std::bind(&PIDControllerNode::ki_callback, this,
                  std::placeholders::_1));
    kd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid/kd", qos_sensor_data,
        std::bind(&PIDControllerNode::kd_callback, this,
                  std::placeholders::_1));
    tau_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(
        control_topic, 10);
    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&PIDControllerNode::publish_tau, this));
    set_pid_params();
}

void PIDControllerNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    eta_ = eta_convert_from_ros_to_eigen(msg);
    nu_ = nu_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::publish_tau() {
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

    std_msgs::msg::Float64MultiArray Kp_msg;
    std_msgs::msg::Float64MultiArray Ki_msg;
    std_msgs::msg::Float64MultiArray Kd_msg;

    Kp_msg.data = Kp_vec;
    Ki_msg.data = Ki_vec;
    Kd_msg.data = Kd_vec;

    pid_controller_.setKp(float64multiarray_to_diagonal_matrix6d(Kp_msg));
    pid_controller_.setKi(float64multiarray_to_diagonal_matrix6d(Ki_msg));
    pid_controller_.setKd(float64multiarray_to_diagonal_matrix6d(Kd_msg));
}

void PIDControllerNode::kp_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    types::Matrix6d Kp = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKp(Kp);
}

void PIDControllerNode::ki_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    types::Matrix6d Ki = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKi(Ki);
}

void PIDControllerNode::kd_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    types::Matrix6d Kd = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKd(Kd);
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

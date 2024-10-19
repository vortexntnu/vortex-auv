#include "pid_controller_dp/pid_controller_ros.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"

PIDControllerNode::PIDControllerNode()
    : Node("pid_controller_node") {
    time_step_ = std::chrono::milliseconds(10);
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/nucleus/odom", 10, std::bind(&PIDControllerNode::odometry_callback, this, std::placeholders::_1));
    kp_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/pid/kp", 10, std::bind(&PIDControllerNode::kp_callback, this, std::placeholders::_1));
    ki_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/pid/ki", 10, std::bind(&PIDControllerNode::ki_callback, this, std::placeholders::_1));
    kd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/pid/kd", 10, std::bind(&PIDControllerNode::kd_callback, this, std::placeholders::_1));
    tau_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/wrench_input", 10);
    tau_pub_timer_ = this->create_wall_timer(time_step_, std::bind(&PIDControllerNode::publish_tau, this));
    eta_ = Eigen::Vector6d::Zero();
    eta_d_ = Eigen::Vector6d::Zero();
    nu_ = Eigen::Vector6d::Zero();
    eta_dot_d_ = Eigen::Vector6d::Zero();
}

void PIDControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d euler_angles = quaternion_to_euler(quat);

    eta_ << x, y, z, euler_angles(0), euler_angles(1), euler_angles(2);

    double u = msg->twist.twist.linear.x;
    double v = msg->twist.twist.linear.y;
    double w = msg->twist.twist.linear.z;
    double p = msg->twist.twist.angular.x;
    double q = msg->twist.twist.angular.y;
    double r = msg->twist.twist.angular.z;

    nu_ << u, v, w, p, q, r;
}

void PIDControllerNode::publish_tau() {
    Eigen::Vector6d tau = pid_controller_.calculate_tau(eta_, eta_d_, nu_, eta_dot_d_);

    geometry_msgs::msg::Wrench tau_msg;
    tau_msg.force.x = tau(0);
    tau_msg.force.y = tau(1);
    tau_msg.force.z = tau(2);
    tau_msg.torque.x = tau(3);
    tau_msg.torque.y = tau(4);
    tau_msg.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void PIDControllerNode::kp_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    Eigen::Matrix6d Kp = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKp(Kp);
}

void PIDControllerNode::ki_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    Eigen::Matrix6d Ki = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKi(Ki);
}

void PIDControllerNode::kd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    Eigen::Matrix6d Kd = float64multiarray_to_diagonal_matrix6d(*msg);
    pid_controller_.setKd(Kd);
}
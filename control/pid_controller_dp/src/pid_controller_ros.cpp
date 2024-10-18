#include "pid_controller_dp/pid_controller_ros.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"

PIDControllerNode::PIDControllerNode()
    : Node("pid_controller_node"),
        pid_controller_(),
        time_step_(10) {
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/nucleus/odom", 10, std::bind(&PIDControllerNode::odometry_callback, this, std::placeholders::_1));
    tau_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/wrench_input", 10);
    tau_pub_timer_ = this->create_wall_timer(time_step_, std::bind(&PIDControllerNode::publish_tau, this));
}

void PIDControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    Eigen::Quaterniond quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d euler_angles = quaternion_to_euler(quat);

    Eigen::Vector6d eta;
    eta << x, y, z, euler_angles(0), euler_angles(1), euler_angles(2);

    pid_controller_.setEta(eta);
}

void PIDControllerNode::publish_tau() {
    Eigen::Vector6d tau = pid_controller_.calculate_tau();

    geometry_msgs::msg::Wrench tau_msg;
    tau_msg.force.x = tau(0);
    tau_msg.force.y = tau(1);
    tau_msg.force.z = tau(2);
    tau_msg.torque.x = tau(3);
    tau_msg.torque.y = tau(4);
    tau_msg.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}
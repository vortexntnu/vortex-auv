#include <reference_filter_dp/reference_filter_ros.hpp>

ReferenceFilterNode::ReferenceFilterNode()
    : Node("reference_filter_node") {
    time_step_ = std::chrono::milliseconds(10);
    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>("/dp/reference", 10);
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/reference/pose", 10, std::bind(&ReferenceFilterNode::reference_callback, this, std::placeholders::_1));
    reference_pub_timer_ = this->create_wall_timer(time_step_, std::bind(&ReferenceFilterNode::reference_publisher_callback, this));
    x_ = Vector18d::Zero();
    }

void ReferenceFilterNode::reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d euler_angles = quaternion_to_euler(quat);

    r_ << x, y, z, euler_angles(0), euler_angles(1), euler_angles(2);
}

void ReferenceFilterNode::reference_publisher_callback() {
    Vector18d x_dot = reference_filter_.calculate_x_dot(x_, r_);

    x_ += x_dot * time_step_.count() / 1000.0;

    vortex_msgs::msg::ReferenceFilter reference_msg;
    
    reference_msg.x = x_(0);
    reference_msg.y = x_(1);
    reference_msg.z = x_(2);
    reference_msg.roll = x_(3);
    reference_msg.pitch = x_(4);
    reference_msg.yaw = x_(5);
    reference_msg.x_dot = x_(6);
    reference_msg.y_dot = x_(7);
    reference_msg.z_dot = x_(8);
    reference_msg.roll_dot = x_(9);
    reference_msg.pitch_dot = x_(10);
    reference_msg.yaw_dot = x_(11);

    reference_pub_->publish(reference_msg);
}


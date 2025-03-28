#include "eskf/eskf_ros.hpp"
#include <iostream>
#include <variant>
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"

ESKFNode::ESKFNode() : Node("eskf_node") {
    time_step = std::chrono::milliseconds(1);
    odom_pub_timer_ = this->create_wall_timer(
        time_step, std::bind(&ESKFNode::publish_odom, this));

    set_subscribers_and_publisher();

    set_parameters();
}

void ESKFNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    this->declare_parameter<std::string>("imu_topic", "imu/data_raw");
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor_data,
        std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_topic", "/orca/twist");
    std::string dvl_topic = this->get_parameter("dvl_topic").as_string();
    dvl_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos_sensor_data,
        std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("odom_topic", "odom");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);
}

void ESKFNode::set_parameters() {
    Eigen::Matrix12d Q;
    Q.setZero();
    Q.diagonal() << sq(0.0103), sq(0.0118), sq(0.0043),  // acceleration noise
        sq(0.00193), sq(0.00306), sq(0.00118),           // gyroscope noise
        sq(0.05), sq(0.05), sq(0.05),  // acceleration bias noise
        sq(0.03), sq(0.03), sq(0.03);  // gyroscope bias noise

    eskf_params_.Q = Q;

    eskf_ = std::make_unique<ESKF>(eskf_params_);

    Eigen::Matrix18d P;
    P.setZero();
    P.diagonal() << 0.1, 0.1, 0.1,  // position
        0.1, 0.1, 0.1,              // velocity
        0.1, 0.1, 0.1,              // euler angles
        0.01, 0.01, 0.01,           // accel bias
        0.01, 0.01, 0.01,           // gyro bias
        0.001, 0.001, 0.001;        // gravity

    error_state_.covariance = P;
}

void ESKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time current_time = msg->header.stamp;

    if (!first_imu_msg_received_) {
        last_imu_time_ = current_time;
        first_imu_msg_received_ = true;
        return;
    }

    double dt = (current_time - last_imu_time_).seconds();
    last_imu_time_ = current_time;

    imu_meas_.accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
    imu_meas_.gyro << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;

    std::tie(nom_state_, error_state_) =
        eskf_->imu_update(nom_state_, error_state_, imu_meas_, dt);
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    dvl_meas_.vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;
    dvl_meas_.cov << msg->twist.covariance[0], msg->twist.covariance[1],
        msg->twist.covariance[2], msg->twist.covariance[6],
        msg->twist.covariance[7], msg->twist.covariance[8],
        msg->twist.covariance[12], msg->twist.covariance[13],
        msg->twist.covariance[14];

    std::tie(nom_state_, error_state_) =
        eskf_->dvl_update(nom_state_, error_state_, dvl_meas_);
}

void ESKFNode::publish_odom() {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.pose.pose.position.x = nom_state_.pos.x();
    odom_msg.pose.pose.position.y = nom_state_.pos.y();
    odom_msg.pose.pose.position.z = nom_state_.pos.z();

    odom_msg.pose.pose.orientation.w = nom_state_.quat.w();
    odom_msg.pose.pose.orientation.x = nom_state_.quat.x();
    odom_msg.pose.pose.orientation.y = nom_state_.quat.y();
    odom_msg.pose.pose.orientation.z = nom_state_.quat.z();

    odom_msg.twist.twist.linear.x = nom_state_.vel.x();
    odom_msg.twist.twist.linear.y = nom_state_.vel.y();
    odom_msg.twist.twist.linear.z = nom_state_.vel.z();

    odom_msg.header.stamp = this->now();  // Add timestamp to the message
    odom_pub_->publish(odom_msg);
}

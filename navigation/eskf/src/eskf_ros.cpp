#include "eskf/eskf_ros.hpp"
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"
#include <spdlog/spdlog.h>

ESKFNode::ESKFNode() : Node("eskf_node") {
    time_step = std::chrono::milliseconds(1);
    odom_pub_timer_ = this->create_wall_timer(time_step, std::bind(&ESKFNode::publish_odom, this));

    set_subscribers_and_publisher();

    set_parameters();

    spdlog::info("ESKF Node Initialized");
}

void ESKFNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    this->declare_parameter<std::string>("imu_topic", "imu/data_raw");
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos_sensor_data, std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_topic", "/orca/twist");
    std::string dvl_topic = this->get_parameter("dvl_topic").as_string();
    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(dvl_topic, qos_sensor_data, std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));


    this->declare_parameter<std::string>("odom_topic", "odom");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, qos_sensor_data);
}

void ESKFNode::set_parameters() {

    std::vector<double> diag_Q_std;
    this->declare_parameter<std::vector<double>>("diag_Q_std");  // gyroscope bias noise

    diag_Q_std = this->get_parameter("diag_Q_std").as_double_array();
    
    Eigen::Matrix12d Q;
    Q.setZero();
    spdlog::info("Q diagonal: {}",diag_Q_std[0]);
    Q.diagonal() << 
        sq(diag_Q_std[0]), sq(diag_Q_std[1]), sq(diag_Q_std[2]),       // acceleration noise
        sq(diag_Q_std[3]), sq(diag_Q_std[4]), sq(diag_Q_std[5]),       // gyroscope noise
        sq(diag_Q_std[6]), sq(diag_Q_std[7]), sq(diag_Q_std[8]),       // acceleration bias noise
        sq(diag_Q_std[9]), sq(diag_Q_std[10]), sq(diag_Q_std[11]);   // gyroscope bias noise
    eskf_params_.Q = Q;

    eskf_ = std::make_unique<ESKF>(eskf_params_);

    Eigen::Matrix18d P;
    P.setZero();
    P.diagonal() << 1.0, 1.0, 1.0,              // position
                    0.1, 0.1, 0.1,              // velocity
                    0.1, 0.1, 0.1,              // euler angles
                    0.001, 0.001, 0.001,        // accel bias
                    0.001, 0.001, 0.001,        // gyro bias
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

    double dt = (current_time - last_imu_time_).nanoseconds() * 1e-9;
    last_imu_time_ = current_time;

    imu_meas_.accel_uncorrected << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu_meas_.gyro_uncorrected << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_meas_.correct();

    std::tie(nom_state_, error_state_) = eskf_->imu_update(nom_state_, error_state_, imu_meas_, dt);
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    dvl_meas_.vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    dvl_meas_.cov << msg->twist.covariance[0], msg->twist.covariance[1], msg->twist.covariance[2], 
        msg->twist.covariance[6], msg->twist.covariance[7], msg->twist.covariance[8],
        msg->twist.covariance[12], msg->twist.covariance[13], msg->twist.covariance[14];

    std::tie(nom_state_, error_state_) = eskf_->dvl_update(nom_state_, error_state_, dvl_meas_);
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

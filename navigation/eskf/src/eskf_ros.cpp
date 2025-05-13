#include "eskf/eskf_ros.hpp"
#include <spdlog/spdlog.h>
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"

ESKFNode::ESKFNode() : Node("eskf_node") {
    time_step = std::chrono::milliseconds(1);
    odom_pub_timer_ = this->create_wall_timer(
        time_step, std::bind(&ESKFNode::publish_odom, this));

    set_subscribers_and_publisher();

    set_parameters();

    spdlog::info("ESKF Node Initialized");
}

void ESKFNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/orca/pose", qos_sensor_data,
        std::bind(&ESKFNode::pose_callback, this, std::placeholders::_1));
    
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/orca/twist", qos_sensor_data,
        std::bind(&ESKFNode::twist_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("imu_topic");
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor_data,
        std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_topic");
    std::string dvl_topic = this->get_parameter("dvl_topic").as_string();
    dvl_sub_ = this->create_subscription<
        stonefish_ros2::msg::DVL>(
        dvl_topic, qos_sensor_data,
        std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("odom_topic");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);

    nis_pub_ = create_publisher<std_msgs::msg::Float64>("dvl/nis", 10);
    nees_pub_ = create_publisher<std_msgs::msg::Float64>("dvl/nees", 10);
}

void ESKFNode::set_parameters() {
    std::vector<double> R_imu_correction;
    this->declare_parameter<std::vector<double>>("imu_frame");
    R_imu_correction = get_parameter("imu_frame").as_double_array();
    R_imu_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        R_imu_correction.data());

    std::vector<double> diag_Q_std;
    this->declare_parameter<std::vector<double>>("diag_Q_std");

    diag_Q_std = this->get_parameter("diag_Q_std").as_double_array();

    Eigen::Matrix12d Q;
    Q.setZero();
    Q.diagonal() << sq(diag_Q_std[0]), sq(diag_Q_std[1]), sq(diag_Q_std[2]),
        sq(diag_Q_std[3]), sq(diag_Q_std[4]), sq(diag_Q_std[5]),
        sq(diag_Q_std[6]), sq(diag_Q_std[7]), sq(diag_Q_std[8]),
        sq(diag_Q_std[9]), sq(diag_Q_std[10]), sq(diag_Q_std[11]);
    eskf_params_.Q = Q;

    eskf_ = std::make_unique<ESKF>(eskf_params_);

    std::vector<double> diag_p_init =
        this->declare_parameter<std::vector<double>>("diag_p_init");
    Eigen::Matrix18d P = createDiagonalMatrix<18>(diag_p_init);

    error_state_.covariance = P;
}

void ESKFNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    g_truth_.pos << msg->pose.pose.position.x,
        msg->pose.pose.position.y, msg->pose.pose.position.z;
    g_truth_.quat.w() = msg->pose.pose.orientation.w;
    g_truth_.quat.x() = msg->pose.pose.orientation.x;
    g_truth_.quat.y() = msg->pose.pose.orientation.y;
    g_truth_.quat.z() = msg->pose.pose.orientation.z;
}

void ESKFNode::twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    g_truth_.vel << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y, msg->twist.twist.linear.z;
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

    Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);

    imu_meas_.accel = R_imu_eskf_ * raw_accel;

    Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);

    imu_meas_.gyro = R_imu_eskf_ * raw_gyro;

    std::tie(nom_state_, error_state_) = eskf_->imu_update(imu_meas_, dt);
}

void ESKFNode::dvl_callback(
    const stonefish_ros2::msg::DVL::SharedPtr msg) {
    dvl_meas_.vel << msg->velocity.x,
        msg->velocity.y, msg->velocity.z;
    dvl_meas_.cov << 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001;
    
    // msg->velocity_covariance[0], msg->velocity_covariance[1], msg->velocity_covariance[2],
    //     msg->velocity_covariance[3], msg->velocity_covariance[4], msg->velocity_covariance[5],
    //    msg->velocity_covariance[6], msg->velocity_covariance[7], msg->velocity_covariance[8];
    

    // Set biases and gravity as float values
    float gyro_bias_x = 0.00001;
    float gyro_bias_y = 0.00001;
    float gyro_bias_z = 0.00001;
    
    float accel_bias_x = 0.00001;
    float accel_bias_y = 0.00001;
    float accel_bias_z = 0.00001;
    
    float gravity_x = 0.0;
    float gravity_y = 0.0;
    float gravity_z = -9.81;
    
    g_truth_.gyro_bias << gyro_bias_x, gyro_bias_y, gyro_bias_z;
    g_truth_.accel_bias << accel_bias_x, accel_bias_y, accel_bias_z;
    g_truth_.gravity << gravity_x, gravity_y, gravity_z;

    eskf_->ground_truth_ = g_truth_;

    std::tie(nom_state_, error_state_) = eskf_->dvl_update(dvl_meas_);

    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->NIS_;
    nis_pub_->publish(nis_msg);

    std_msgs::msg::Float64 nees_msg;
    nees_msg.data = eskf_->NEES_;
    nees_pub_->publish(nees_msg);
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

    odom_msg.header.stamp = this->now();
    odom_pub_->publish(odom_msg);
}

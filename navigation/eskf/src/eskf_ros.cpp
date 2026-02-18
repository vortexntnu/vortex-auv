#include "eskf/eskf_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "eskf/typedefs.hpp"

auto start_message{R"(
     ________   ______   ___  ____   ________
    |_   __  |.' ____ \ |_  ||_  _| |_   __  |
      | |_ \_|| (___ \_|  | |_/ /     | |_ \_|
      |  _| _  _.____`.   |  __'.     |  _|
     _| |__/ || \____) | _| |  \ \_  _| |_
    |________| \______.'|____||____||_____|
)"};

ESKFNode::ESKFNode(const rclcpp::NodeOptions& options)
    : Node("eskf_node", options) {
    time_step = std::chrono::milliseconds(1);
    odom_pub_timer_ = this->create_wall_timer(
        time_step, std::bind(&ESKFNode::publish_odom, this));

    set_subscribers_and_publisher();

    set_parameters();

    spdlog::info(start_message);

#ifndef NDEBUG
    spdlog::info(
        "______________________Debug mode is enabled______________________");
#endif
}

void ESKFNode::set_subscribers_and_publisher() {
    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);
    this->declare_parameter<std::string>("imu_topic");
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor_data,
        std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_topic");
    std::string dvl_topic = this->get_parameter("dvl_topic").as_string();
    dvl_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos_sensor_data,
        std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("odom_topic");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);

#ifndef NDEBUG
    nis_pub_ = create_publisher<std_msgs::msg::Float64>(
        "eskf/nis", vortex::utils::qos_profiles::reliable_profile());
#endif
}

void ESKFNode::set_parameters() {
    std::vector<double> R_imu_correction;
    this->declare_parameter<std::vector<double>>("imu_frame");
    R_imu_correction = get_parameter("imu_frame").as_double_array();
    R_imu_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        R_imu_correction.data());

    std::vector<double> R_dvl_correction;
    this->declare_parameter<std::vector<double>>("dvl_frame");
    R_dvl_correction = get_parameter("dvl_frame").as_double_array();
    R_dvl_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        R_dvl_correction.data());

    std::vector<double> diag_Q_std;
    this->declare_parameter<std::vector<double>>("diag_Q_std");

    diag_Q_std = this->get_parameter("diag_Q_std").as_double_array();

    if (diag_Q_std.size() != 12) {
        throw std::runtime_error("diag_Q_std must have length 12");
    }

    Eigen::Matrix12d Q = Eigen::Map<const Eigen::Vector12d>(diag_Q_std.data())
                             .array()
                             .square()
                             .matrix()
                             .asDiagonal();

    std::vector<double> diag_p_init =
        this->declare_parameter<std::vector<double>>("diag_p_init");
    if (diag_p_init.size() != 15) {
        throw std::runtime_error("diag_p_init must have length 15");
    }
    Eigen::Matrix15d P = createDiagonalMatrix<15>(diag_p_init);

    EskfParams eskf_params{.Q = Q, .P = P};

    eskf_ = std::make_unique<ESKF>(eskf_params);
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

    ImuMeasurement imu_measurement{};
    imu_measurement.accel = R_imu_eskf_ * raw_accel;

    Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);

    // currently the gyro and the accelorometer are rotated differently.
    // should be changed with the actual drone params.
    // imu_measurement.gyro = R_imu_eskf_ * raw_gyro;
    imu_measurement.gyro = raw_gyro;

    // used for publishing odom output of eskf
    latest_gyro_measurement_ = imu_measurement.gyro;

    eskf_->imu_update(imu_measurement, dt);
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    SensorDVL dvl_sensor;
    dvl_sensor.measurement << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y, msg->twist.twist.linear.z;

    dvl_sensor.measurement_noise << msg->twist.covariance[0],
        msg->twist.covariance[1], msg->twist.covariance[2],
        msg->twist.covariance[6], msg->twist.covariance[7],
        msg->twist.covariance[8], msg->twist.covariance[12],
        msg->twist.covariance[13], msg->twist.covariance[14];

    // Apply the rotation correction to the DVL measurement
    dvl_sensor.measurement = R_dvl_eskf_ * dvl_sensor.measurement;
    dvl_sensor.measurement_noise =
        R_dvl_eskf_ * dvl_sensor.measurement_noise * R_dvl_eskf_.transpose();

    eskf_->dvl_update(dvl_sensor);

#ifndef NDEBUG
    // Publish NIS in Debug mode
    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->get_nis();
    nis_pub_->publish(nis_msg);
#endif
}

void ESKFNode::publish_odom() {
    nav_msgs::msg::Odometry odom_msg;
    StateQuat nom_state = eskf_->get_nominal_state();
    StateEuler error_state_ = eskf_->get_error_state();

    odom_msg.pose.pose.position.x = nom_state.pos.x();
    odom_msg.pose.pose.position.y = nom_state.pos.y();
    odom_msg.pose.pose.position.z = nom_state.pos.z();

    odom_msg.pose.pose.orientation.w = nom_state.quat.w();
    odom_msg.pose.pose.orientation.x = nom_state.quat.x();
    odom_msg.pose.pose.orientation.y = nom_state.quat.y();
    odom_msg.pose.pose.orientation.z = nom_state.quat.z();

    odom_msg.twist.twist.linear.x = nom_state.vel.x();
    odom_msg.twist.twist.linear.y = nom_state.vel.y();
    odom_msg.twist.twist.linear.z = nom_state.vel.z();

    // Add bias values to the angular velocity field of twist
    Eigen::Vector3d body_angular_vel =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    odom_msg.twist.twist.angular.x = body_angular_vel.x();
    odom_msg.twist.twist.angular.y = body_angular_vel.y();
    odom_msg.twist.twist.angular.z = body_angular_vel.z();

    // If you also want to include gyro bias, you could add it to the covariance
    // matrix or publish a separate topic for biases

    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";

    // Some cross terms of the covariance are ignored, and the acc/gyro biases
    // cov are not published. Pos and orientation cov needs to be mapped from
    // 6*6 matrix to an array (states 0-2)

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            odom_msg.pose.covariance[i * 6 + j] = error_state_.covariance(i, j);
        }
    }

    // Orientation covariance (states 6–8)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            odom_msg.pose.covariance[(i + 3) * 6 + (j + 3)] =
                error_state_.covariance(i + 6, j + 6);
        }
    }

    // Linear velocity covariance
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            odom_msg.twist.covariance[i * 6 + j] =
                error_state_.covariance(i + 3, j + 3);
        }
    }
    odom_pub_->publish(odom_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ESKFNode)

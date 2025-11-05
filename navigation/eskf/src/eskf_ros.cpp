#include "eskf/eskf_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include "eskf/eskf_utils.hpp"
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
        spdlog::info("______________________Debug mode is enabled______________________");
    #endif
}

void ESKFNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

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

    nis_pub_ = create_publisher<std_msgs::msg::Float64>("dvl/nis", 10);

    error_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("eskf/error", 10);

    odom_cov_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("eskf/odom_covariance", 10);

    odom_ground_truth_ =  this->create_subscription<nav_msgs::msg::Odometry>(
        "/orca/odom", qos_sensor_data,
        std::bind(&ESKFNode::ground_truth_callback, this,
                  std::placeholders::_1));
    // incoming_imu_eskf_ = this->create_publisher<sensor_msgs::msg::Imu>("incoming_imu_eskf", rclcpp::SensorDataQoS() );
    #endif
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
    Eigen::Matrix15d P = create_diagonal_matrix<15>(diag_p_init);

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

    Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);

    // imu_meas_.accel = R_imu_eskf_ * raw_accel;
    imu_meas_.accel = -raw_accel;

    Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);

    // imu_meas_.gyro = R_imu_eskf_ * raw_gyro;
    imu_meas_.gyro = raw_gyro;

    std::tie(nom_state_, error_state_) = eskf_->imu_update(imu_meas_, dt);

    // to remove
    // sensor_msgs::msg::Imu imu_out;
    // imu_out.header = msg->header;
    // imu_out.linear_acceleration.x = imu_meas_.accel.x();
    // imu_out.linear_acceleration.y = imu_meas_.accel.y();
    // imu_out.linear_acceleration.z = imu_meas_.accel.z();
    // imu_out.angular_velocity.x = imu_meas_.gyro.x();
    // imu_out.angular_velocity.y = imu_meas_.gyro.y();
    // imu_out.angular_velocity.z = imu_meas_.gyro.z();
    // incoming_imu_eskf_->publish(imu_out);
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    dvl_sensor_.measurement << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;

    dvl_sensor_.measurement_noise << msg->twist.covariance[0], msg->twist.covariance[1],
        msg->twist.covariance[2], msg->twist.covariance[6],
        msg->twist.covariance[7], msg->twist.covariance[8],
        msg->twist.covariance[12], msg->twist.covariance[13],
        msg->twist.covariance[14];

    std::tie(nom_state_, error_state_) = eskf_->dvl_update(dvl_sensor_);

    #ifndef NDEBUG
    // Publish NIS in Debug mode
    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->NIS_;
    nis_pub_->publish(nis_msg);
    #endif
}

#ifndef NDEBUG
void ESKFNode::ground_truth_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    // Position error
    last_error_.pos.x() = nom_state_.pos.x() - msg->pose.pose.position.x;
    last_error_.pos.y() = nom_state_.pos.y() - msg->pose.pose.position.y;
    last_error_.pos.z() = nom_state_.pos.z() - msg->pose.pose.position.z;

    // Velocity error
    last_error_.vel.x() = nom_state_.vel.x() - msg->twist.twist.linear.x;
    last_error_.vel.y() = nom_state_.vel.y() - msg->twist.twist.linear.y;
    last_error_.vel.z() = nom_state_.vel.z() - msg->twist.twist.linear.z;

    // Orientation error
    Eigen::Quaterniond q_nom(nom_state_.quat.x(), nom_state_.quat.y(),
                          nom_state_.quat.z(), nom_state_.quat.w());
    Eigen::Quaterniond q_gt(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);

    Eigen::Quaterniond q_err = q_gt.inverse() * q_nom;  // error rotation
    last_error_.ori = quaternion_to_euler(q_err);

    // Publish
    std_msgs::msg::Float64MultiArray error_msg;
    error_msg.data = { last_error_.pos.x(), last_error_.pos.y(), last_error_.pos.z(),
                       last_error_.vel.x(), last_error_.vel.y(), last_error_.vel.z(),
                       last_error_.ori.x(), last_error_.ori.y(), last_error_.ori.z() };
    error_pub_->publish(error_msg);
}
#endif

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

    // Add bias values to the angular velocity field of twist
    odom_msg.twist.twist.angular.x = nom_state_.accel_bias.x();
    odom_msg.twist.twist.angular.y = nom_state_.accel_bias.y();
    odom_msg.twist.twist.angular.z = nom_state_.accel_bias.z();

    // If you also want to include gyro bias, you could add it to the covariance
    // matrix or publish a separate topic for biases

    odom_msg.header.stamp = this->now();
    odom_pub_->publish(odom_msg);


    // publish the covariance matrix
    std_msgs::msg::Float64MultiArray msg;

    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "rows";
    msg.layout.dim[0].size = 15;
    msg.layout.dim[0].stride = 15 * 15;
    msg.layout.dim[1].label = "cols";
    msg.layout.dim[1].size = 15;
    msg.layout.dim[1].stride = 15;

    // Flatten Eigen matrix into a std::vector<double>
    msg.data.resize(15 * 15);
    msg.data.assign(
        error_state_.covariance.data(),
        error_state_.covariance.data() + 15 * 15
    );

    // Publish
    odom_cov_pub_->publish(msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ESKFNode)

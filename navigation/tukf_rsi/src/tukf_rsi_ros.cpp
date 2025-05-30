#include "tukf_node.hpp"
#include <cmath>

TUKFNode::TUKFNode()
: Node("tukf_node")
{

    odom_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&TUKFNode::publishOdom, this));
        
    set_subscribers_and_publisher();

    set_parameters();
    
    spdlog::info("TUKF Node Initialized");
}

void TUKFNode::set_subscribers_and_publisher() {
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    this->declare_parameter<std::string>("gyro_topic");
    std::string gyro_topic = this->get_parameter("gyro_topic").as_string();
    gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        gyro_topic, qos,
        std::bind(&TUKFNode::gyroCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_topic");
    std::string dvl_topic = this->get_parameter("dvl_topic").as_string();
    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos,
        std::bind(&TUKFNode::dvlCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("wrench_topic");
    std::string dvl_topic = this->get_parameter("wrench_topic").as_string();
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        wrench_sub_, qos,
        std::bind(&TUKFNode::wrenchCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("odom_topic");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos);
}

void TUKFNode::set_parameters() {
    this->declare_parameter<std::vector<double>>("diag_Q_std");
    auto diagQ = this->get_parameter("diag_Q_std").as_double_array();
    Eigen::Matrix37d Q = Eigen::Matrix37d::Zero();
    for (int i = 0; i < 37; ++i) {
        Q(i,i) = diagQ[i] * diagQ[i];
    }

    this->declare_parameter<std::vector<double>>("diag_P0_std");
    auto diagP0 = this->get_parameter("diag_P0_std").as_double_array();
    Eigen::Matrix37d P0 = Eigen::Matrix37d::Zero();
    for (int i = 0; i < 37; ++i) {
        P0(i,i) = diagP0[i] * diagP0[i];
    }

    this->declare_parameter<std::vector<double>>("x0");
    auto x0_vec = this->get_parameter("x0").as_double_array();
    Eigen::Vector37d x0_e;
    for (int i = 0; i < 37; ++i) x0_e[i] = x0_vec[i];

    this->declare_parameter<double>("dt", 0.01);
    dt_ = this->get_parameter("dt").as_double();

    this->declare_parameter<std::vector<double>>("diag_Rgyro_std");
    auto diagRgyro = this->get_parameter("diag_Rgyro_std").as_double_array();
    R_gyro_ = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) R_gyro_(i,i) = diagRgyro[i] * diagRgyro[i];

    this->declare_parameter<std::vector<double>>("diag_Rdvl_std");
    auto diagRdvl = this->get_parameter("diag_Rdvl_std").as_double_array();
    R_dvl_ = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) R_dvl_(i,i) = diagRdvl[i] * diagRdvl[i];

    tukf_ = std::make_unique<TUKF>(x0, Q, dt_);
    tukf_->x.covariance = P0;
    tukf_->x.fill_states(x0_e);
}

void TUKFNode::gyro_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Vector3d gyro(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);
    MeasModel m;
    m.measurement = gyro;
    m.covariance = R_gyro_;
    m.H = [](const AUVState& s) {
        MeasModel mm;
        Eigen::Matrix3x12d Hm = Eigen::Matrix3x12d::Zero();
        Hm.block<3,3>(0,9) = Eigen::Matrix3d::Identity();
        mm.measurement = Hm * s.dynamic_part();
        return mm;
    };

    tukf_->measurement_update(state_, m);
    state_ = tukf_->posterior_estimate(state_, m);
}

void TUKFNode::dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    Eigen::Vector3d vel(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    Eigen::Matrix3d cov;
    cov << msg->twist.covariance[0], msg->twist.covariance[1], msg->twist.covariance[2],
           msg->twist.covariance[6], msg->twist.covariance[7], msg->twist.covariance[8],
           msg->twist.covariance[12], msg->twist.covariance[13], msg->twist.covariance[14];
    MeasModel m;
    m.measurement = vel;
    m.covariance = R_dvl_;

    tukf_->measurement_update(state_, m);
    state_ = tukf_->posterior_estimate(state_, m);
}

void TUKFNode::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    Eigen::Vector6d wrench_input(msg->wrench.force.x,
                          msg->wrench.force.y,
                          msg->wrench.force.z,
                          msg->wrench.torque.x,
                          msg->wrench.torque.y,
                          msg->wrench.torque.z);

    state_ = tukf_->unscented_transform(state_, control_force);
}

void TUKFNode::publish_odom() {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = state_.position.x();
    odom.pose.pose.position.y = state_.position.y();
    odom.pose.pose.position.z = state_.position.z();
    odom.pose.pose.orientation.w = state_.orientation.w();
    odom.pose.pose.orientation.x = state_.orientation.x();
    odom.pose.pose.orientation.y = state_.orientation.y();
    odom.pose.pose.orientation.z = state_.orientation.z();

    odom.twist.twist.linear.x = state_.velocity.x();
    odom.twist.twist.linear.y = state_.velocity.y();
    odom.twist.twist.linear.z = state_.velocity.z();
    odom.twist.twist.angular.x = state_.angular_velocity.x();
    odom.twist.twist.angular.y = state_.angular_velocity.y();
    odom.twist.twist.angular.z = state_.angular_velocity.z();

    odom_pub_->publish(odom);
}

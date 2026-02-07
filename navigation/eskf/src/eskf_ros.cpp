#include "eskf/eskf_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "eskf/typedefs.hpp"
#include <std_msgs/msg/float64.hpp>

auto start_message{R"(
     ________   ______   ___  ____   ________
    |_   __  |.' ____ \ |_  ||_  _| |_   __  |
      | |_ \_|| (___ \_|  | |_/ /     | |_ \_|
      |  _| _  _.____`.   |  __'.     |  _|
     _| |__/ || \____) | _| |  \ \_  _| |_
    |________| \______.'|____||____||_____|
)"};

static inline void pub_f64(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub,
    double v) {
    if (!pub) return;
    std_msgs::msg::Float64 m;
    m.data = v;
    pub->publish(m);
}

static inline void pub_i32(
    const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& pub,
    int v) {
    if (!pub) return;
    std_msgs::msg::Int32 m;
    m.data = v;
    pub->publish(m);
}

static inline void pub_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& pub,
    bool v) {
    if (!pub) return;
    std_msgs::msg::Bool m;
    m.data = v;
    pub->publish(m);
}

ESKFNode::ESKFNode(const rclcpp::NodeOptions& options)
    : Node("eskf_node", options) {
    // time_step = std::chrono::milliseconds(1);
    // odom_pub_timer_ = this->create_wall_timer(
    //     time_step, std::bind(&ESKFNode::publish_odom, this));

    set_subscribers_and_publisher();

    set_parameters();

    setup_services();

    spdlog::info(start_message);
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

    nis_pub_ = create_publisher<std_msgs::msg::Float64>(
        "dvl/nis", vortex::utils::qos_profiles::reliable_profile());

    pub_vo_rejects_ = create_publisher<std_msgs::msg::Int32>(
        "eskf/debug/vo_consecutive_rejects", vortex::utils::qos_profiles::reliable_profile());

    pub_vo_anchor_valid_ = create_publisher<std_msgs::msg::Bool>(
        "eskf/debug/vo_anchor_valid", vortex::utils::qos_profiles::reliable_profile());

        
    this->declare_parameter<std::string>("pose_topic", "/landmark/pose_cov");
    std::string pose_topic = this->get_parameter("pose_topic").as_string();

    visualEgomotion_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&ESKFNode::visualEgomotion_callback, this, std::placeholders::_1));
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
    if (diag_p_init.size() != 18) {
        throw std::runtime_error("diag_p_init must have length 18");
    }
    Eigen::Matrix18d P = createDiagonalMatrix<18>(diag_p_init);
    EskfParams eskf_params{.Q = Q, .P = P};

    eskf_ = std::make_unique<ESKF>(eskf_params);

        this->declare_parameter<double>("vo_reset_gap_sec", 1.0);
    double vo_gap = this->get_parameter("vo_reset_gap_sec").as_double();
    eskf_->set_vo_reset_gap(vo_gap);
    
    this->declare_parameter<bool>("disable_nis_gating", false);
    bool disable_gating = this->get_parameter("disable_nis_gating").as_bool();
    eskf_->set_nis_gating_enabled(!disable_gating);
    
    if (disable_gating) { // For testing
        RCLCPP_WARN(this->get_logger(), 
            "NIS gating disabled");
    }
    
    RCLCPP_INFO(this->get_logger(), "VO reset gap: %.2f s", vo_gap);
}

void ESKFNode::setup_services() {
    toggle_gating_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/toggle_nis_gating",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            if (eskf_) {
                eskf_->set_nis_gating_enabled(request->data);
                response->success = true;
                response->message = request->data ? 
                    "NIS gating ENABLED" : 
                    "NIS gating DISABLED";
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            } else {
                response->success = false;
                response->message = "ESKF not initialized";
            }
        });

    reset_anchor_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/force_anchor_reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            if (eskf_) {
                eskf_->force_anchor_reset();
                response->success = true;
                response->message = "VO anchor will reset on next measurement";
                RCLCPP_INFO(this->get_logger(), "Anchor reset forced via service");
            } else {
                response->success = false;
                response->message = "ESKF not initialized";
            }
        });
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

    imu_measurement.gyro = R_imu_eskf_ * raw_gyro;

    eskf_->imu_update(imu_measurement, dt);
    publish_odom();
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    DvlMeasurement dvl_measurement{};
    dvl_measurement.vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;

    dvl_measurement.cov << msg->twist.covariance[0], msg->twist.covariance[1],
        msg->twist.covariance[2],  msg->twist.covariance[6],
        msg->twist.covariance[7],  msg->twist.covariance[8],
        msg->twist.covariance[12], msg->twist.covariance[13],
        msg->twist.covariance[14];

    eskf_->dvl_update(dvl_measurement);

    std_msgs::msg::Float64 nis_msg;
    nis_msg.data = eskf_->get_nis();
    nis_pub_->publish(nis_msg);

    publish_odom();
}

void ESKFNode::visualEgomotion_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    VisualMeasurement visual_meas{};

    visual_meas.pos << msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z;

    visual_meas.quat = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    visual_meas.R = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        msg->pose.covariance.data());

    visual_meas.stamp_sec = rclcpp::Time(msg->header.stamp).seconds();

    eskf_->visualEgomotion_update(visual_meas);
    
    pub_i32(pub_vo_rejects_, eskf_->get_consecutive_vo_rejects());
    pub_bool(pub_vo_anchor_valid_, eskf_->is_anchor_valid());

    publish_odom();
}

void ESKFNode::publish_odom() {
    nav_msgs::msg::Odometry odom_msg;
    StateQuat nom_state = eskf_->get_nominal_state();

    odom_msg.pose.pose.position.x    = nom_state.pos.x();
    odom_msg.pose.pose.position.y    = nom_state.pos.y();
    odom_msg.pose.pose.position.z    = nom_state.pos.z();

    odom_msg.pose.pose.orientation.w = nom_state.quat.w();
    odom_msg.pose.pose.orientation.x = nom_state.quat.x();
    odom_msg.pose.pose.orientation.y = nom_state.quat.y();
    odom_msg.pose.pose.orientation.z = nom_state.quat.z();

    odom_msg.twist.twist.linear.x    = nom_state.vel.x();
    odom_msg.twist.twist.linear.y    = nom_state.vel.y();
    odom_msg.twist.twist.linear.z    = nom_state.vel.z();

    odom_msg.twist.twist.angular.x   = nom_state.accel_bias.x();
    odom_msg.twist.twist.angular.y   = nom_state.accel_bias.y();
    odom_msg.twist.twist.angular.z   = nom_state.accel_bias.z();

    // TODO: Covariance out

    odom_msg.header.stamp = this->now();
    odom_pub_->publish(odom_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ESKFNode)

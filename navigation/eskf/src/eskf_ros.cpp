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

ESKFNode::ESKFNode(const rclcpp::NodeOptions& options)
    : Node("eskf_node", options),
      first_imu_msg_received_(false),
      use_vo_(false),
      last_marker_id_(INVALID_ID),
      have_last_marker_(false) {
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

    this->declare_parameter<bool>("vo.use_vo");
    this->declare_parameter<std::string>("vo.landmarks_topic");
    this->declare_parameter<std::string>("vo.base_frame");
    this->declare_parameter<std::string>("vo.camera_frame");
    this->declare_parameter<bool>("vo.disable_gating");

    this->declare_parameter<double>("vo.nis_gate_pose");
    this->declare_parameter<double>("vo.nis_gate_velocity");
    this->declare_parameter<double>("vo.dropout_timeout_sec");
    this->declare_parameter<int>("vo.rejects_limit");
    this->declare_parameter<double>("vo.pos_floor");
    this->declare_parameter<double>("vo.att_floor");
    this->declare_parameter<double>("vo.vel_floor");
    this->declare_parameter<double>("vo.vel_alpha");
    this->declare_parameter<double>("vo.dt_min");
    this->declare_parameter<double>("vo.dt_max");
    this->declare_parameter<bool>("vo.use_sw");
    this->declare_parameter<int>("vo.window_size");
    this->declare_parameter<double>("vo.max_age");
    this->declare_parameter<double>("vo.huber_deg");
    this->declare_parameter<double>("vo.gate_deg");

    use_vo_ = this->get_parameter("vo.use_vo").as_bool();
    vo_base_frame_ = this->get_parameter("vo.base_frame").as_string();
    vo_cam_frame_ = this->get_parameter("vo.camera_frame").as_string();

    VoConfig vo_cfg;
    vo_cfg.nis_gate_pose       = this->get_parameter("vo.nis_gate_pose").as_double();
    vo_cfg.nis_gate_vel        = this->get_parameter("vo.nis_gate_velocity").as_double();
    vo_cfg.dropout_timeout_sec = this->get_parameter("vo.dropout_timeout_sec").as_double();
    vo_cfg.rejects_limit       = this->get_parameter("vo.rejects_limit").as_int();
    vo_cfg.pos_floor           = this->get_parameter("vo.pos_floor").as_double();
    vo_cfg.att_floor           = this->get_parameter("vo.att_floor").as_double();
    vo_cfg.vel_floor           = this->get_parameter("vo.vel_floor").as_double();
    vo_cfg.vel_alpha           = this->get_parameter("vo.vel_alpha").as_double();
    vo_cfg.dt_min              = this->get_parameter("vo.dt_min").as_double();
    vo_cfg.dt_max              = this->get_parameter("vo.dt_max").as_double();
    vo_cfg.use_sw              = this->get_parameter("vo.use_sw").as_bool();
    vo_cfg.sw_window_size      = this->get_parameter("vo.window_size").as_int();
    vo_cfg.sw_max_age          = this->get_parameter("vo.max_age").as_double();
    vo_cfg.sw_huber_deg        = this->get_parameter("vo.huber_deg").as_double();
    vo_cfg.sw_gate_deg         = this->get_parameter("vo.gate_deg").as_double();

    eskf_->set_vo_enabled(use_vo_);
    eskf_->set_nis_gating_enabled(!this->get_parameter("vo.disable_gating").as_bool());
    eskf_->set_vo_config(vo_cfg);

    if (use_vo_) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::string landmarks_topic = this->get_parameter("vo.landmarks_topic").as_string();
        landmark_sub_ = this->create_subscription<vortex_msgs::msg::LandmarkArray>(
            landmarks_topic, rclcpp::SensorDataQoS(),
            std::bind(&ESKFNode::landmark_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "VO enabled: %s -> %s",
            vo_cam_frame_.c_str(), vo_base_frame_.c_str());
    }

    if (this->get_parameter("vo.disable_gating").as_bool()) {
        RCLCPP_WARN(this->get_logger(), "NIS gating disabled");
    }
}

void ESKFNode::setup_services() {
    toggle_gating_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/toggle_nis_gating",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            eskf_->set_nis_gating_enabled(req->data);
            res->success = true;
            res->message = req->data ? "NIS gating enabled" : "NIS gating disabled";
        });

    reset_anchor_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/force_anchor_reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            eskf_->force_anchor_reset();
            res->success = true;
            res->message = "Anchor reset";
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

void ESKFNode::landmark_callback(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
    if (!msg || msg->landmarks.empty()) return;

    std::vector<const vortex_msgs::msg::Landmark*> markers;
    for (const auto& lm : msg->landmarks) {
        if (lm.type == vortex_msgs::msg::Landmark::ARUCO_MARKER) {
            markers.push_back(&lm);
        }
    }
    if (markers.empty()) return;

    auto it = std::min_element(markers.begin(), markers.end(),
        [](const auto* a, const auto* b) { return a->subtype < b->subtype; });
    const auto* chosen = *it;
    uint16_t chosen_id = chosen->subtype;

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        tf_msg = tf_buffer_->lookupTransform(vo_base_frame_, vo_cam_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "TF %s <- %s: %s", vo_base_frame_.c_str(), vo_cam_frame_.c_str(), ex.what());
        return;
    }

    Eigen::Vector3d t_base_cam(tf_msg.transform.translation.x,
                               tf_msg.transform.translation.y,
                               tf_msg.transform.translation.z);
    Eigen::Quaterniond q_base_cam(tf_msg.transform.rotation.w,
                                  tf_msg.transform.rotation.x,
                                  tf_msg.transform.rotation.y,
                                  tf_msg.transform.rotation.z);

    Eigen::Vector3d t_cam_marker(chosen->pose.pose.position.x,
                                 chosen->pose.pose.position.y,
                                 chosen->pose.pose.position.z);
    Eigen::Quaterniond q_cam_marker(chosen->pose.pose.orientation.w,
                                    chosen->pose.pose.orientation.x,
                                    chosen->pose.pose.orientation.y,
                                    chosen->pose.pose.orientation.z);

    Eigen::Vector3d t_base_marker = t_base_cam + q_base_cam * t_cam_marker;
    Eigen::Quaterniond q_base_marker = (q_base_cam * q_cam_marker).normalized();

    if (have_last_marker_ && chosen_id != last_marker_id_) {
        RCLCPP_INFO(this->get_logger(), "Marker switch %u -> %u", last_marker_id_, chosen_id);
        eskf_->handle_marker_switch(t_base_marker, q_base_marker);
    }
    last_marker_id_ = chosen_id;
    have_last_marker_ = true;

    VisualMeasurement meas;
    meas.pos = t_base_marker;
    meas.quat = q_base_marker;
    meas.stamp_ = rclcpp::Time(msg->header.stamp).seconds();
    meas.R = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        chosen->pose.covariance.data());

    eskf_->landmark_update(meas);

    std_msgs::msg::Int32 rejects_msg;
    rejects_msg.data = eskf_->get_consecutive_rejects();
    pub_vo_rejects_->publish(rejects_msg);

    std_msgs::msg::Bool anchor_msg;
    anchor_msg.data = eskf_->is_anchor_valid();
    pub_vo_anchor_valid_->publish(anchor_msg);

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

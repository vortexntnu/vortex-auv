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

    use_tf_transforms_ = this->declare_parameter<bool>("use_tf_transforms");
    tf_sensors_loaded_ = !use_tf_transforms_;

    frame_prefix_ = this->declare_parameter<std::string>("frame_prefix", "");
    if (!frame_prefix_.empty() && frame_prefix_.back() == '/') {
        frame_prefix_.pop_back();
    }
    spdlog::info("frame_prefix set to '{}'", frame_prefix_);

    publish_tf_ = this->declare_parameter<bool>("publish_tf");
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    publish_pose_ = this->declare_parameter<bool>("publish_pose");
    publish_twist_ = this->declare_parameter<bool>("publish_twist");

    // Declare these here so they appear in `ros2 param list` from startup,
    // even though they are read in complete_initialization().
    this->declare_parameter<int>("publish_rate_ms");
    this->declare_parameter<std::string>("topics.imu");
    this->declare_parameter<std::string>("topics.dvl_twist");
    this->declare_parameter<std::string>("topics.odom");
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");

    if (use_tf_transforms_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ESKFNode::lookup_static_transforms, this));
    } else {
        spdlog::info(
            "Using parameter-based sensor transforms. TF lookup disabled.");
        complete_initialization();
    }
}

void ESKFNode::set_subscribers_and_publisher() {
    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);

    std::string imu_topic = this->get_parameter("topics.imu").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor_data,
        std::bind(&ESKFNode::imu_callback, this, std::placeholders::_1));

    std::string dvl_topic = this->get_parameter("topics.dvl_twist").as_string();
    dvl_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos_sensor_data,
        std::bind(&ESKFNode::dvl_callback, this, std::placeholders::_1));

    std::string odom_topic = this->get_parameter("topics.odom").as_string();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);

    if (publish_pose_) {
        std::string pose_topic = this->get_parameter("topics.pose").as_string();
        pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                pose_topic, qos_sensor_data);
    }

    if (publish_twist_) {
        std::string twist_topic =
            this->get_parameter("topics.twist").as_string();
        twist_pub_ =
            this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                twist_topic, qos_sensor_data);
    }

#ifndef NDEBUG
    nis_pub_ = create_publisher<std_msgs::msg::Float64>(
        "eskf/nis", vortex::utils::qos_profiles::reliable_profile());
#endif
}

void ESKFNode::set_parameters() {

    if (!use_tf_transforms_) {
        std::vector<double> R_imu_correction =
            this->declare_parameter<std::vector<double>>("transform.imu_frame_r");
        R_imu_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_imu_correction.data());

        std::vector<double> T_imu_correction =
            this->declare_parameter<std::vector<double>>("transform.imu_frame_t");
        T_imu_eskf_ = Eigen::Map<Eigen::Vector3d>(T_imu_correction.data());

        std::vector<double> R_dvl_correction =
            this->declare_parameter<std::vector<double>>("transform.dvl_frame_r");
        R_dvl_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_dvl_correction.data());

        std::vector<double> T_dvl_correction =
            this->declare_parameter<std::vector<double>>("transform.dvl_frame_t");
        T_dvl_eskf_ = Eigen::Map<Eigen::Vector3d>(T_dvl_correction.data());

        std::vector<double> T_depth_correction =
            this->declare_parameter<std::vector<double>>("transform.depth_frame_t");
        T_depth_eskf_ = Eigen::Map<Eigen::Vector3d>(T_depth_correction.data());
    }

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

    std::vector<double> initial_gyro_bias =
        this->declare_parameter<std::vector<double>>(
            "initial_gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
        spdlog::info("initial_gyro_bias: [{}, {}, {}]", initial_gyro_bias[0],
                     initial_gyro_bias[1], initial_gyro_bias[2]);
    if (initial_gyro_bias.size() != 3) {
        throw std::runtime_error("initial_gyro_bias must have length 3");
    }

    std::vector<double> initial_accel_bias =
        this->declare_parameter<std::vector<double>>(
            "initial_accel_bias", std::vector<double>{0.0, 0.0, 0.0});
        spdlog::info("initial_accel_bias: [{}, {}, {}]", initial_accel_bias[0],
                     initial_accel_bias[1], initial_accel_bias[2]);
    if (initial_accel_bias.size() != 3) {
        throw std::runtime_error("initial_accel_bias must have length 3");
    }

    EskfParams eskf_params{
        .Q = Q,
        .P = P,
        .initial_gyro_bias =
            Eigen::Map<Eigen::Vector3d>(initial_gyro_bias.data()),
        .initial_accel_bias =
            Eigen::Map<Eigen::Vector3d>(initial_accel_bias.data())};

    eskf_ = std::make_unique<ESKF>(eskf_params);

    add_gravity_to_imu_ =
        this->declare_parameter<bool>("add_gravity_to_imu");
    spdlog::info("add_gravity_to_imu: {}", add_gravity_to_imu_);
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

    ImuMeasurement imu_measurement{};

    Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);

    Eigen::Vector3d raw_gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                             msg->angular_velocity.z);

    Eigen::Vector3d accel_aligned = R_imu_eskf_ * raw_accel;

    // currently the gyro and the accelorometer are rotated differently in sim.
    // should be changed with the actual drone params.
    // Eigen::Vector3d gyro_aligned = R_imu_eskf_ * raw_gyro;
    Eigen::Vector3d gyro_aligned = raw_gyro;
    imu_measurement.gyro = gyro_aligned;

    // lever arm correction for accelerometer
    StateQuat nom_state = eskf_->get_nominal_state();
    Eigen::Vector3d omega = gyro_aligned - nom_state.gyro_bias;

    // a_corrected = a_meas - omega x (omega x T)
    Eigen::Vector3d centripetal_accel = omega.cross(omega.cross(T_imu_eskf_));
    accel_aligned -= centripetal_accel;

    if (add_gravity_to_imu_) {
        Eigen::Matrix3d R = nom_state.quat.normalized().toRotationMatrix();
        accel_aligned -= R.transpose() * eskf_->get_gravity();
    }

    imu_measurement.accel = accel_aligned;

    // save latest gyro readings (used for DVL correction and odom output)
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

    // Apply the rotation and translation corrections to the DVL measurement
    StateQuat nom_state = eskf_->get_nominal_state();
    // get the angular velocity
    Eigen::Vector3d omega_corrected =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    // correct rotation and translation: v_base = v_sensor - omega x T
    dvl_sensor.measurement = R_dvl_eskf_ * dvl_sensor.measurement -
                             omega_corrected.cross(T_dvl_eskf_);
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

    // publishing the velocity in the body frame
    Eigen::Matrix3d R_body_to_world = nom_state.quat.toRotationMatrix();

    Eigen::Vector3d v_body = R_body_to_world.transpose() * nom_state.vel;

    odom_msg.twist.twist.linear.x = v_body.x();
    odom_msg.twist.twist.linear.y = v_body.y();
    odom_msg.twist.twist.linear.z = v_body.z();

    // Add bias values to the angular velocity field of twist
    Eigen::Vector3d body_angular_vel =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    odom_msg.twist.twist.angular.x = body_angular_vel.x();
    odom_msg.twist.twist.angular.y = body_angular_vel.y();
    odom_msg.twist.twist.angular.z = body_angular_vel.z();

    // If you also want to include gyro bias, you could add it to the covariance
    // matrix or publish a separate topic for biases
    rclcpp::Time current_time = this->now();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame("odom");

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

    if (publish_pose_) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose;
        pose_pub_->publish(pose_msg);
    }

    if (publish_twist_) {
        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = odom_msg.header;
        twist_msg.twist = odom_msg.twist;
        twist_pub_->publish(twist_msg);
    }

    if (publish_tf_) {
        publish_tf(nom_state, current_time);
    }
}

void ESKFNode::lookup_static_transforms() {
    try {
        Tf_base_imu_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame("imu_link"), tf2::TimePointZero));
        R_imu_eskf_ = Tf_base_imu_.rotation();
        T_imu_eskf_ = Tf_base_imu_.translation();

        Tf_base_dvl_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame("dvl_link"), tf2::TimePointZero));
        R_dvl_eskf_ = Tf_base_dvl_.rotation();
        T_dvl_eskf_ = Tf_base_dvl_.translation();

        Tf_base_depth_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame("pressure_sensor_link"), tf2::TimePointZero));
        T_depth_eskf_ = Tf_base_depth_.translation();

        tf_sensors_loaded_ = true;
        tf_timer_->cancel();
        spdlog::info("All static transforms loaded successfully.");
        complete_initialization();
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("TF Lookup failed (will retry): {}", ex.what());
    }
}

void ESKFNode::complete_initialization() {
    set_subscribers_and_publisher();
    set_parameters();

    time_step_ = std::chrono::milliseconds(
        this->get_parameter("publish_rate_ms").as_int());
    odom_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&ESKFNode::publish_odom, this));

    spdlog::info(start_message);

#ifndef NDEBUG
    spdlog::info(
        "______________________Debug mode is enabled______________________");
#endif
}

void ESKFNode::publish_tf(const StateQuat& nom_state, const rclcpp::Time& time) {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = frame("odom");
    tf_msg.child_frame_id = frame("base_link");

    tf_msg.transform.translation.x = nom_state.pos.x();
    tf_msg.transform.translation.y = nom_state.pos.y();
    tf_msg.transform.translation.z = nom_state.pos.z();

    tf_msg.transform.rotation.w = nom_state.quat.w();
    tf_msg.transform.rotation.x = nom_state.quat.x();
    tf_msg.transform.rotation.y = nom_state.quat.y();
    tf_msg.transform.rotation.z = nom_state.quat.z();

    tf_broadcaster_->sendTransform(tf_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ESKFNode)

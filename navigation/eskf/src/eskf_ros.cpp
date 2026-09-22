#include "eskf/eskf_ros.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include "eskf/output.hpp"
#ifdef ESKF_WITH_GTSAM
#include "eskf/gtsam_navigation.hpp"
#endif
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
    frame_prefix_ = this->declare_parameter<std::string>("frame_prefix", "");
    if (!frame_prefix_.empty() && frame_prefix_.back() == '/') {
        frame_prefix_.pop_back();
    }
    spdlog::info("frame_prefix set to '{}'", frame_prefix_);

    publish_debug_ = this->declare_parameter<bool>("publish_debug");
    if (publish_debug_) {
        spdlog::info(
            "Debug output enabled: Publishing ESKF outputs on debug/private "
            "topics and disabling TF publishing.");
    } else {
        spdlog::info(
            "Debug output disabled: Publishing ESKF outputs on standard topics "
            "and enabling TF publishing.");
    }

    publish_tf_ =
        this->declare_parameter<bool>("publish_tf") && !publish_debug_;
    if (publish_tf_) {
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    publish_pose_ = this->declare_parameter<bool>("publish_pose");
    publish_twist_ = this->declare_parameter<bool>("publish_twist");

    publish_biases_ = this->declare_parameter<bool>("publish_biases");

    this->declare_parameter<int>("publish_rate_ms");
    this->declare_parameter<std::string>("topics.imu");
    this->declare_parameter<std::string>("topics.dvl_twist");
    this->declare_parameter<std::string>("topics.pressure_sensor");
    this->declare_parameter<std::string>("topics.odom");
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");

    imu_use_tf_transform_ =
        this->declare_parameter<bool>("sensors.imu.use_tf_transform");
    dvl_use_tf_transform_ =
        this->declare_parameter<bool>("sensors.dvl.use_tf_transform");
    dvl_use_msg_noise_ =
        this->declare_parameter<bool>("sensors.dvl.use_msg_noise");
    pressure_use_tf_transform_ =
        this->declare_parameter<bool>("sensors.pressure.use_tf_transform");
    pressure_use_msg_noise_ =
        this->declare_parameter<bool>("sensors.pressure.use_msg_noise");

    const bool any_use_tf = imu_use_tf_transform_ || dvl_use_tf_transform_ ||
                            pressure_use_tf_transform_;

    if (any_use_tf) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    auto qos_sensor_data = rclcpp::SensorDataQoS().keep_last(100);

    std::string imu_topic = this->get_parameter("topics.imu").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_sensor_data,
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            imu_callback(msg);
        });

    std::string dvl_topic = this->get_parameter("topics.dvl_twist").as_string();
    dvl_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        dvl_topic, qos_sensor_data,
        [this](
            const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
                msg) { dvl_callback(msg); });

    std::string pressure_topic =
        this->get_parameter("topics.pressure_sensor").as_string();
    depth_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        pressure_topic, qos_sensor_data,
        [this](const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
            pressure_callback(msg);
        });

    validity_pub_ = create_publisher<std_msgs::msg::Bool>("eskf/valid", 1);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "eskf/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            reset_estimator();
            first_imu_msg_received_ = propagated_ = dvl_received_ =
                depth_received_ = faulted_ = false;
            last_dvl_stamp_ = last_depth_stamp_ = -1;
            response->success = true;
            response->message =
                "Navigation reset to configured prior; awaiting IMU, DVL and "
                "depth";
        });

    auto eskf_debug_topic = [](std::string& topic_name) {
        const std::string prefix = "eskf/";

        if (topic_name.rfind(prefix, 0) != 0) {
            topic_name = prefix + topic_name;
        }
    };

    std::string odom_topic = this->get_parameter("topics.odom").as_string();
    if (publish_debug_) {
        eskf_debug_topic(odom_topic);
    }
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);

    if (publish_pose_) {
        std::string pose_topic = this->get_parameter("topics.pose").as_string();
        if (publish_debug_) {
            eskf_debug_topic(pose_topic);
        }
        pose_pub_ = this->create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic,
                                                           qos_sensor_data);
    }

    if (publish_twist_) {
        std::string twist_topic =
            this->get_parameter("topics.twist").as_string();
        if (publish_debug_) {
            eskf_debug_topic(twist_topic);
        }
        twist_pub_ = this->create_publisher<
            geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic,
                                                            qos_sensor_data);
    }

    if (publish_biases_) {
        accel_bias_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "eskf/accel_bias", qos_sensor_data);
        gyro_bias_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "eskf/gyro_bias", qos_sensor_data);
    }

    if (publish_nis_) {
        nis_dvl_pub_ = create_publisher<std_msgs::msg::Float64>(
            "eskf/nis_dvl", qos_sensor_data);
        nis_depth_pub_ = create_publisher<std_msgs::msg::Float64>(
            "eskf/nis_depth", qos_sensor_data);
    }

    if (publish_debug_) {
        dvl_body_pub_ =
            create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "eskf/dvl_body", qos_sensor_data);
        depth_pub_ = create_publisher<std_msgs::msg::Float64>("eskf/depth",
                                                              qos_sensor_data);
    }
}

void ESKFNode::set_parameters() {
    if (!imu_use_tf_transform_) {
        std::vector<double> R_imu =
            this->declare_parameter<std::vector<double>>(
                "sensors.imu.transform.r");
        if (R_imu.size() != 9)
            throw std::runtime_error("Invalid R_imu transform size");
        R_imu_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_imu.data());

        std::vector<double> T_imu =
            this->declare_parameter<std::vector<double>>(
                "sensors.imu.transform.t");
        if (T_imu.size() != 3)
            throw std::runtime_error("Invalid T_imu transform size");
        T_imu_eskf_ = Eigen::Map<Eigen::Vector3d>(T_imu.data());
    }

    if (!dvl_use_tf_transform_) {
        std::vector<double> R_dvl =
            this->declare_parameter<std::vector<double>>(
                "sensors.dvl.transform.r");
        if (R_dvl.size() != 9)
            throw std::runtime_error("Invalid R_dvl transform size");
        R_dvl_eskf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_dvl.data());

        std::vector<double> T_dvl =
            this->declare_parameter<std::vector<double>>(
                "sensors.dvl.transform.t");
        if (T_dvl.size() != 3)
            throw std::runtime_error("Invalid T_dvl transform size");
        T_dvl_eskf_ = Eigen::Map<Eigen::Vector3d>(T_dvl.data());
    }

    if (!pressure_use_tf_transform_) {
        std::vector<double> T_depth =
            this->declare_parameter<std::vector<double>>(
                "sensors.pressure.transform.t");
        if (T_depth.size() != 3)
            throw std::runtime_error("Invalid T_depth transform size");
        T_depth_eskf_ = Eigen::Map<Eigen::Vector3d>(T_depth.data());
    }

    if (!dvl_use_msg_noise_) {
        auto diag = this->declare_parameter<std::vector<double>>(
            "sensors.dvl.measurement_noise_std_diag");
        if (diag.size() != 3) {
            throw std::runtime_error(
                "sensors.dvl.measurement_noise_std_diag must have length 3");
        }
        dvl_measurement_noise_std_ = Eigen::Vector3d(diag[0], diag[1], diag[2]);
    }

    {
        pressure_measurement_noise_ = this->declare_parameter<double>(
            "sensors.pressure.measurement_noise");
    }

    std::vector<double> diag_Q_std;
    this->declare_parameter<std::vector<double>>("diag_Q_std");

    diag_Q_std = this->get_parameter("diag_Q_std").as_double_array();

    if (diag_Q_std.size() != 12 ||
        !std::all_of(diag_Q_std.begin(), diag_Q_std.end(),
                     [](double x) { return std::isfinite(x) && x >= 0; })) {
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

    Eigen::Vector3d g_vec(0.0, 0.0, gravity_);

    EskfParams eskf_params{
        .Q = Q,
        .P = P,
        .g_ = g_vec,
    };

    max_imu_dt_ = declare_parameter<double>("max_imu_dt", 0.1);
    max_estimate_age_ = declare_parameter<double>("max_estimate_age", 0.25);
    max_aiding_skew_ = declare_parameter<double>("max_aiding_skew", 0.05);
    if (!std::isfinite(max_estimate_age_) || max_estimate_age_ <= 0 ||
        !std::isfinite(max_aiding_skew_) || max_aiding_skew_ < 0 ||
        !std::isfinite(pressure_measurement_noise_) ||
        pressure_measurement_noise_ <= 0 ||
        !dvl_measurement_noise_std_.allFinite() ||
        (dvl_measurement_noise_std_.array() <= 0).any())
        throw std::runtime_error(
            "Invalid timing or sensor noise configuration");
    for (const auto& R : {R_imu_eskf_, R_dvl_eskf_}) {
        if (!R.allFinite() ||
            !(R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
            std::abs(R.determinant() - 1) > 1e-6)
            throw std::runtime_error("Sensor transform must be a rotation");
    }
    if (!T_imu_eskf_.allFinite() || !T_dvl_eskf_.allFinite() ||
        !T_depth_eskf_.allFinite())
        throw std::runtime_error("Invalid sensor translation");
    eskf_params.max_imu_dt = max_imu_dt_;
    eskf_params.dvl_nis_threshold =
        declare_parameter<double>("dvl_nis_threshold", 16.266);
    eskf_params.depth_nis_threshold =
        declare_parameter<double>("depth_nis_threshold", 10.828);
    estimator_backend_ =
        declare_parameter<std::string>("estimator_backend", "eskf");
    smoother_lag_ = declare_parameter<double>("smoother_lag", 2.0);
    keyframe_interval_ = declare_parameter<double>("keyframe_interval", 0.2);
    filter_params_ = eskf_params;
    reset_estimator();
}

void ESKFNode::reset_estimator() {
    if (estimator_backend_ == "eskf") {
        eskf_ = std::make_unique<ESKF>(filter_params_);
        return;
    }
#ifdef ESKF_WITH_GTSAM
    if (estimator_backend_ == "gtsam") {
        GtsamNavigationParams options;
        options.lag_seconds = smoother_lag_;
        options.keyframe_interval = keyframe_interval_;
        eskf_ = std::make_unique<GtsamNavigation>(filter_params_, options);
        return;
    }
#endif
    throw std::runtime_error(
        "Unknown/unavailable estimator_backend; build with ESKF_WITH_GTSAM for "
        "gtsam");
}

void ESKFNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    if (faulted_)
        return;
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age < 0 || age > max_estimate_age_)
        return;
    const Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                                    msg->linear_acceleration.y,
                                    msg->linear_acceleration.z);
    const Eigen::Vector3d raw_gyro(msg->angular_velocity.x,
                                   msg->angular_velocity.y,
                                   msg->angular_velocity.z);
    if (!raw_accel.allFinite() || !raw_gyro.allFinite() ||
        msg->angular_velocity_covariance[0] < 0 ||
        msg->linear_acceleration_covariance[0] < 0)
        return;
    const Eigen::Vector3d gyro = R_imu_eskf_ * raw_gyro;
    if (!first_imu_msg_received_) {
        last_imu_time_ = stamp;
        previous_gyro_ = latest_gyro_measurement_ = gyro;
        first_imu_msg_received_ = true;
        return;
    }
    const double dt = (stamp - last_imu_time_).seconds();
    if (dt <= 0)
        return;
    if (dt > max_imu_dt_) {
        faulted_ = true;
        RCLCPP_ERROR(get_logger(),
                     "IMU gap exceeds limit; call eskf/reset to establish a "
                     "new navigation origin");
        return;
    }
    const auto nominal = eskf_->get_nominal_state();
    const Eigen::Vector3d omega = gyro - nominal.gyro_bias;
    const Eigen::Vector3d alpha = (gyro - previous_gyro_) / dt;
    ImuMeasurement measurement;
    measurement.gyro = gyro;
    measurement.accel = R_imu_eskf_ * raw_accel -
                        omega.cross(omega.cross(T_imu_eskf_)) -
                        alpha.cross(T_imu_eskf_);
    if (!eskf_->imu_update(measurement, dt)) {
        faulted_ = true;
        RCLCPP_ERROR(get_logger(), "Estimator propagation failed: %s",
                     eskf_->error_message().c_str());
        return;
    }
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
        gyro_cov(msg->angular_velocity_covariance.data());
    if (gyro_cov.allFinite() &&
        gyro_cov.isApprox(gyro_cov.transpose(), 1e-10) &&
        gyro_cov.diagonal().minCoeff() > 0 &&
        gyro_cov.llt().info() == Eigen::Success)
        gyro_covariance_ = R_imu_eskf_ * gyro_cov * R_imu_eskf_.transpose();
    else
        gyro_covariance_ =
            filter_params_.Q.block<3, 3>(NoiseIndex::gyro, NoiseIndex::gyro) /
            dt;
    previous_gyro_ = latest_gyro_measurement_ = gyro;
    last_imu_time_ = stamp;
    propagated_ = true;
}

bool ESKFNode::accept_aiding_stamp(const rclcpp::Time& stamp,
                                   double& previous_stamp) {
    if (!propagated_ || faulted_)
        return false;
    const double age = (now() - stamp).seconds();
    const double skew = (stamp - last_imu_time_).seconds();
    // No rewind in the ESKF: only accept measurements close to its current
    // epoch.
    if (age < 0 || age > max_estimate_age_ ||
        std::abs(skew) > max_aiding_skew_ || stamp.seconds() <= previous_stamp)
        return false;
    previous_stamp = stamp.seconds();
    return true;
}

void ESKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
    if (!accept_aiding_stamp(
            rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()),
            last_dvl_stamp_))
        return;
    SensorDVL dvl_sensor;

    dvl_sensor.measurement << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y, msg->twist.twist.linear.z;

    if (dvl_use_msg_noise_) {
        dvl_sensor.measurement_noise << msg->twist.covariance[0],
            msg->twist.covariance[1], msg->twist.covariance[2],
            msg->twist.covariance[6], msg->twist.covariance[7],
            msg->twist.covariance[8], msg->twist.covariance[12],
            msg->twist.covariance[13], msg->twist.covariance[14];
    } else {
        dvl_sensor.measurement_noise =
            dvl_measurement_noise_std_.array().square().matrix().asDiagonal();
    }

    // Apply the rotation and translation corrections to the DVL measurement
    NominalState nom_state = eskf_->get_nominal_state();
    // get the angular velocity
    Eigen::Vector3d omega_corrected =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    // correct rotation and translation: v_base = v_sensor - omega x T
    dvl_sensor.measurement = R_dvl_eskf_ * dvl_sensor.measurement -
                             omega_corrected.cross(T_dvl_eskf_);
    dvl_sensor.measurement_noise =
        R_dvl_eskf_ * dvl_sensor.measurement_noise * R_dvl_eskf_.transpose();

    if (publish_debug_) {
        geometry_msgs::msg::TwistWithCovarianceStamped dvl_body_msg;
        dvl_body_msg.header.stamp = msg->header.stamp;
        dvl_body_msg.header.frame_id = frame("base_link");
        dvl_body_msg.twist.twist.linear.x = dvl_sensor.measurement.x();
        dvl_body_msg.twist.twist.linear.y = dvl_sensor.measurement.y();
        dvl_body_msg.twist.twist.linear.z = dvl_sensor.measurement.z();
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
            dvl_body_msg.twist.covariance.data())
            .topLeftCorner<3, 3>() = dvl_sensor.measurement_noise;
        dvl_body_pub_->publish(dvl_body_msg);
    }

    const bool accepted = eskf_->dvl_update(dvl_sensor);
    dvl_received_ = dvl_received_ || accepted;
    if (!eskf_->healthy()) {
        faulted_ = true;
        RCLCPP_ERROR(get_logger(), "Estimator failed: %s",
                     eskf_->error_message().c_str());
    }
    if (!accepted)
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "DVL correction rejected (invalid covariance/data or innovation)");

    if (publish_nis_) {
        std_msgs::msg::Float64 nis_msg;
        nis_msg.data = eskf_->get_nis_dvl();
        nis_dvl_pub_->publish(nis_msg);
    }
}

void ESKFNode::pressure_callback(
    const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
    if (!accept_aiding_stamp(
            rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()),
            last_depth_stamp_))
        return;
    if (!std::isfinite(msg->fluid_pressure) || !std::isfinite(msg->variance) ||
        msg->variance < 0)
        return;
    SensorDepth depth_sensor;
    const double p_gauge = pressure_is_gauge_
                               ? msg->fluid_pressure
                               : msg->fluid_pressure - atmospheric_pressure_;
    depth_sensor.measurement = p_gauge / (water_density_ * gravity_);
    depth_sensor.lever_arm = T_depth_eskf_;

    const double pressure_variance =
        pressure_use_msg_noise_ && msg->variance > 0.0
            ? msg->variance
            : pressure_measurement_noise_;

    depth_sensor.measurement_noise =
        pressure_variance / std::pow(water_density_ * gravity_, 2);

    if (publish_debug_) {
        std_msgs::msg::Float64 depth_msg;
        depth_msg.data = depth_sensor.measurement;
        depth_pub_->publish(depth_msg);
    }

    const bool accepted = eskf_->depth_update(depth_sensor);
    depth_received_ = depth_received_ || accepted;
    if (!eskf_->healthy()) {
        faulted_ = true;
        RCLCPP_ERROR(get_logger(), "Estimator failed: %s",
                     eskf_->error_message().c_str());
    }
    if (!accepted)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Depth correction rejected (invalid "
                             "covariance/data or innovation)");

    if (publish_nis_) {
        std_msgs::msg::Float64 nis_msg;
        nis_msg.data = eskf_->get_nis_depth();
        nis_depth_pub_->publish(nis_msg);
    }
}

void ESKFNode::publish_odom() {
    std_msgs::msg::Bool validity;
    const double age =
        first_imu_msg_received_ ? (now() - last_imu_time_).seconds() : -1;
    validity.data = propagated_ && dvl_received_ && depth_received_ &&
                    !faulted_ && eskf_->healthy() && age >= 0 &&
                    age <= max_estimate_age_;
    validity_pub_->publish(validity);
    if (!validity.data)
        return;
    nav_msgs::msg::Odometry odom_msg;
    NominalState nom_state = eskf_->get_nominal_state();
    ErrorState error_state_ = eskf_->get_error_state();

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

    // Publish bias-corrected body angular velocity.
    Eigen::Vector3d body_angular_vel =
        latest_gyro_measurement_ - nom_state.gyro_bias;
    odom_msg.twist.twist.angular.x = body_angular_vel.x();
    odom_msg.twist.twist.angular.y = body_angular_vel.y();
    odom_msg.twist.twist.angular.z = body_angular_vel.z();

    // If you also want to include gyro bias, you could add it to the covariance
    // matrix or publish a separate topic for biases
    rclcpp::Time current_time = last_imu_time_;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame("odom");

    odom_msg.child_frame_id = frame("base_link");
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        odom_msg.pose.covariance.data()) =
        eskf_output::pose_covariance(nom_state, error_state_.covariance);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        odom_msg.twist.covariance.data()) =
        eskf_output::twist_covariance(nom_state, error_state_.covariance,
                                      gyro_covariance_);
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
        twist_msg.header.frame_id = frame("base_link");
        twist_msg.twist = odom_msg.twist;
        twist_pub_->publish(twist_msg);
    }

    if (publish_tf_) {
        publish_tf(nom_state, current_time);
    }

    if (publish_biases_) {
        geometry_msgs::msg::Vector3Stamped accel_bias_msg;
        accel_bias_msg.header.stamp = current_time;
        accel_bias_msg.header.frame_id =
            frame("base_link");  // Biases are in the body frame

        accel_bias_msg.vector.x = nom_state.accel_bias.x();
        accel_bias_msg.vector.y = nom_state.accel_bias.y();
        accel_bias_msg.vector.z = nom_state.accel_bias.z();

        accel_bias_pub_->publish(accel_bias_msg);

        geometry_msgs::msg::Vector3Stamped gyro_bias_msg;
        gyro_bias_msg.header = accel_bias_msg.header;

        gyro_bias_msg.vector.x = nom_state.gyro_bias.x();
        gyro_bias_msg.vector.y = nom_state.gyro_bias.y();
        gyro_bias_msg.vector.z = nom_state.gyro_bias.z();

        gyro_bias_pub_->publish(gyro_bias_msg);
    }
}

void ESKFNode::lookup_static_transforms() {
    try {
        if (imu_use_tf_transform_) {
            Tf_base_imu_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
                frame("base_link"), frame("imu_link"), tf2::TimePointZero));
            R_imu_eskf_ = Tf_base_imu_.rotation();
            T_imu_eskf_ = Tf_base_imu_.translation();
        }

        if (dvl_use_tf_transform_) {
            Tf_base_dvl_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
                frame("base_link"), frame("dvl_link"), tf2::TimePointZero));
            R_dvl_eskf_ = Tf_base_dvl_.rotation();
            T_dvl_eskf_ = Tf_base_dvl_.translation();
        }

        if (pressure_use_tf_transform_) {
            Tf_base_depth_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
                frame("base_link"), frame("pressure_sensor_link"),
                tf2::TimePointZero));
            T_depth_eskf_ = Tf_base_depth_.translation();
        }

        tf_timer_->cancel();
        spdlog::info("All required static transforms loaded successfully.");
        complete_initialization();
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("TF lookup failed (will retry): {}", ex.what());
    }
}

void ESKFNode::complete_initialization() {
    publish_nis_ = this->declare_parameter<bool>("publish_nis", false);
    this->gravity_ = this->declare_parameter<double>("gravity.acceleration");
    this->water_density_ = this->declare_parameter<double>("water.density");
    this->atmospheric_pressure_ =
        this->declare_parameter<double>("atmosphere.pressure");
    this->pressure_is_gauge_ =
        this->declare_parameter<bool>("pressure_is_gauge");
    if (!std::isfinite(gravity_) || gravity_ <= 0 ||
        !std::isfinite(water_density_) || water_density_ <= 0 ||
        !std::isfinite(atmospheric_pressure_))
        throw std::runtime_error("Invalid pressure conversion parameters");
    set_parameters();
    set_subscribers_and_publisher();

    time_step_ = std::chrono::milliseconds(
        this->get_parameter("publish_rate_ms").as_int());
    if (time_step_.count() <= 0)
        throw std::runtime_error("publish_rate_ms must be positive");
    odom_pub_timer_ =
        this->create_wall_timer(time_step_, [this]() { publish_odom(); });

    spdlog::info(start_message);

    if (publish_nis_) {
        spdlog::info(
            "NIS publishing enabled on eskf/nis_dvl and eskf/nis_depth");
    }
}

void ESKFNode::publish_tf(const NominalState& nom_state,
                          const rclcpp::Time& time) {
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

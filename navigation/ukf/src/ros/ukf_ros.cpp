#include "ukf/ros/ukf_ros.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "ukf/ros/output.hpp"

UKFNode::UKFNode(const rclcpp::NodeOptions& options)
    : Node("ukf_node", options) {
    frame_prefix_ = this->declare_parameter<std::string>("frame_prefix", "");
    if (!frame_prefix_.empty() && frame_prefix_.back() == '/') {
        frame_prefix_.pop_back();
    }
    spdlog::info("frame_prefix set to '{}'", frame_prefix_);

    publish_debug_ = this->declare_parameter<bool>("publish_debug");
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
    this->declare_parameter<std::string>("topics.magnetometer", "");
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
    magnetometer_use_tf_transform_ =
        this->declare_parameter<bool>("sensors.magnetometer.use_tf_transform");
    pressure_use_msg_noise_ =
        this->declare_parameter<bool>("sensors.pressure.use_msg_noise");

    const bool any_use_tf = imu_use_tf_transform_ || dvl_use_tf_transform_ ||
                            pressure_use_tf_transform_ ||
                            magnetometer_use_tf_transform_;

    if (any_use_tf) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { lookup_static_transforms(); });
    } else {
        spdlog::info(
            "Using parameter-based sensor transforms. TF lookup disabled.");
        complete_initialization();
    }
}

void UKFNode::set_parameters() {
    if (!imu_use_tf_transform_) {
        std::vector<double> R_imu =
            this->declare_parameter<std::vector<double>>(
                "sensors.imu.transform.r");
        if (R_imu.size() != 9)
            throw std::runtime_error("Invalid R_imu transform size");
        R_imu_ukf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_imu.data());

        std::vector<double> T_imu =
            this->declare_parameter<std::vector<double>>(
                "sensors.imu.transform.t");
        if (T_imu.size() != 3)
            throw std::runtime_error("Invalid T_imu transform size");
        T_imu_ukf_ = Eigen::Map<Eigen::Vector3d>(T_imu.data());
    }

    if (!dvl_use_tf_transform_) {
        std::vector<double> R_dvl =
            this->declare_parameter<std::vector<double>>(
                "sensors.dvl.transform.r");
        if (R_dvl.size() != 9)
            throw std::runtime_error("Invalid R_dvl transform size");
        R_dvl_ukf_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            R_dvl.data());

        std::vector<double> T_dvl =
            this->declare_parameter<std::vector<double>>(
                "sensors.dvl.transform.t");
        if (T_dvl.size() != 3)
            throw std::runtime_error("Invalid T_dvl transform size");
        T_dvl_ukf_ = Eigen::Map<Eigen::Vector3d>(T_dvl.data());
    }

    if (!magnetometer_use_tf_transform_) {
        std::vector<double> R_magnetometer =
            this->declare_parameter<std::vector<double>>(
                "sensors.magnetometer.transform.r");
        if (R_magnetometer.size() != 9)
            throw std::runtime_error("Invalid R_magnetometer transform size");
        R_magnetometer_ukf_ =
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                R_magnetometer.data());
    }

    if (!pressure_use_tf_transform_) {
        std::vector<double> T_depth =
            this->declare_parameter<std::vector<double>>(
                "sensors.pressure.transform.t");
        if (T_depth.size() != 3)
            throw std::runtime_error("Invalid T_depth transform size");
        T_depth_ukf_ = Eigen::Map<Eigen::Vector3d>(T_depth.data());
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

    pressure_measurement_noise_ =
        this->declare_parameter<double>("sensors.pressure.measurement_noise");

    std::vector<double> diag_Q_std =
        this->declare_parameter<std::vector<double>>("diag_Q_std");
    if (diag_Q_std.size() != 12 ||
        !std::all_of(diag_Q_std.begin(), diag_Q_std.end(),
                     [](double x) { return std::isfinite(x) && x >= 0; })) {
        throw std::runtime_error("diag_Q_std must have length 12");
    }

    ins_config_.noise_psd =
        Eigen::Map<const Eigen::Vector12d>(diag_Q_std.data())
            .array()
            .square()
            .matrix()
            .asDiagonal();
    ins_config_.gravity = Eigen::Vector3d(0.0, 0.0, gravity_);

    std::vector<double> diag_p_init =
        this->declare_parameter<std::vector<double>>("diag_p_init");
    if (diag_p_init.size() != 15) {
        throw std::runtime_error("diag_p_init must have length 15");
    }
    prior_ = StateGaussian{
        .mean = State{}, .covariance = create_diagonal_matrix<15>(diag_p_init)};
    if (!prior_.covariance.allFinite() ||
        !prior_.covariance.ldlt().isPositive()) {
        throw std::runtime_error("diag_p_init must be positive definite");
    }

    transform_config_.alpha =
        this->declare_parameter<double>("unscented_transform.alpha");
    transform_config_.beta =
        this->declare_parameter<double>("unscented_transform.beta");
    transform_config_.kappa =
        this->declare_parameter<double>("unscented_transform.kappa");

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
    for (const auto& R : {R_imu_ukf_, R_dvl_ukf_, R_magnetometer_ukf_}) {
        if (!R.allFinite() ||
            !(R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
            std::abs(R.determinant() - 1) > 1e-6)
            throw std::runtime_error("Sensor transform must be a rotation");
    }
    if (!T_imu_ukf_.allFinite() || !T_dvl_ukf_.allFinite() ||
        !T_depth_ukf_.allFinite())
        throw std::runtime_error("Invalid sensor translation");

    dvl_nis_threshold_ = declare_parameter<double>("dvl_nis_threshold");
    depth_nis_threshold_ =
        declare_parameter<double>("depth_nis_threshold");
    magnetometer_nis_threshold_ =
        declare_parameter<double>("magnetometer_nis_threshold");

    // the magnetometer noise and reference field are fixed, so it is built once
    const std::vector<double> magnetometer_noise_std =
        this->declare_parameter<std::vector<double>>(
            "sensors.magnetometer.measurement_noise_std_diag");
    const std::vector<double> reference_field =
        this->declare_parameter<std::vector<double>>(
            "sensors.magnetometer.reference_field");
    if (magnetometer_noise_std.size() != 3 || reference_field.size() != 3) {
        throw std::runtime_error(
            "magnetometer noise and reference field must have length 3");
    }
    magnetometer_model_ =
        std::make_unique<MagnetometerMeasurementModel>(MagnetometerConfig{
            .field_noise =
                Eigen::Map<const Eigen::Vector3d>(magnetometer_noise_std.data())
                    .array()
                    .square()
                    .matrix()
                    .asDiagonal(),
            .reference_field =
                Eigen::Map<const Eigen::Vector3d>(reference_field.data())});

    ukf_ = std::make_unique<UnscentedKalmanFilter>(StrapdownINS3D(ins_config_),
                                                   transform_config_);
    estimate_ = prior_;
}

void UKFNode::set_subscribers_and_publishers() {
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
    pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        pressure_topic, qos_sensor_data,
        [this](const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
            pressure_callback(msg);
        });

    const std::string magnetometer_topic =
        this->get_parameter("topics.magnetometer").as_string();
    if (!magnetometer_topic.empty()) {
        magnetometer_sub_ =
            this->create_subscription<sensor_msgs::msg::MagneticField>(
                magnetometer_topic, qos_sensor_data,
                [this](
                    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
                    magnetometer_callback(msg);
                });
    }

    validity_pub_ = create_publisher<std_msgs::msg::Bool>("ukf/valid", 1);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "ukf/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            reset_estimate();
            response->success = true;
            response->message =
                "Navigation reset to configured prior; awaiting IMU, DVL and "
                "depth";
        });

    // outputs go under ukf/ when debugging, so they can run beside the eskf
    auto ukf_debug_topic = [](std::string& topic_name) {
        const std::string prefix = "ukf/";
        if (topic_name.rfind(prefix, 0) != 0) {
            topic_name = prefix + topic_name;
        }
    };

    std::string odom_topic = this->get_parameter("topics.odom").as_string();
    if (publish_debug_) {
        ukf_debug_topic(odom_topic);
    }
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data);

    if (publish_pose_) {
        std::string pose_topic = this->get_parameter("topics.pose").as_string();
        if (publish_debug_) {
            ukf_debug_topic(pose_topic);
        }
        pose_pub_ = this->create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic,
                                                           qos_sensor_data);
    }

    if (publish_twist_) {
        std::string twist_topic =
            this->get_parameter("topics.twist").as_string();
        if (publish_debug_) {
            ukf_debug_topic(twist_topic);
        }
        twist_pub_ = this->create_publisher<
            geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic,
                                                            qos_sensor_data);
    }

    if (publish_biases_) {
        accel_bias_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "ukf/accel_bias", qos_sensor_data);
        gyro_bias_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "ukf/gyro_bias", qos_sensor_data);
    }

    if (publish_nis_) {
        nis_dvl_pub_ = create_publisher<std_msgs::msg::Float64>(
            "ukf/nis_dvl", qos_sensor_data);
        nis_depth_pub_ = create_publisher<std_msgs::msg::Float64>(
            "ukf/nis_depth", qos_sensor_data);
        nis_magnetometer_pub_ = create_publisher<std_msgs::msg::Float64>(
            "ukf/nis_magnetometer", qos_sensor_data);
    }

    if (publish_debug_) {
        dvl_body_pub_ =
            create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "ukf/dvl_body", qos_sensor_data);
        depth_pub_ = create_publisher<std_msgs::msg::Float64>("ukf/depth",
                                                              qos_sensor_data);
    }
}

void UKFNode::lookup_static_transforms() {
    try {
        if (imu_use_tf_transform_) {
            const Eigen::Isometry3d Tf_base_imu =
                tf2::transformToEigen(tf_buffer_->lookupTransform(
                    frame("base_link"), frame("imu_link"), tf2::TimePointZero));
            R_imu_ukf_ = Tf_base_imu.rotation();
            T_imu_ukf_ = Tf_base_imu.translation();
        }

        if (dvl_use_tf_transform_) {
            const Eigen::Isometry3d Tf_base_dvl =
                tf2::transformToEigen(tf_buffer_->lookupTransform(
                    frame("base_link"), frame("dvl_link"), tf2::TimePointZero));
            R_dvl_ukf_ = Tf_base_dvl.rotation();
            T_dvl_ukf_ = Tf_base_dvl.translation();
        }

        if (pressure_use_tf_transform_) {
            const Eigen::Isometry3d Tf_base_depth =
                tf2::transformToEigen(tf_buffer_->lookupTransform(
                    frame("base_link"), frame("pressure_sensor_link"),
                    tf2::TimePointZero));
            T_depth_ukf_ = Tf_base_depth.translation();
        }

        if (magnetometer_use_tf_transform_) {
            // the magnetometer is the one in the Nucleus, next to the dvl
            const Eigen::Isometry3d Tf_base_magnetometer =
                tf2::transformToEigen(tf_buffer_->lookupTransform(
                    frame("base_link"), frame("dvl_link"),
                    tf2::TimePointZero));
            R_magnetometer_ukf_ = Tf_base_magnetometer.rotation();
        }

        tf_timer_->cancel();
        spdlog::info("All required static transforms loaded successfully.");
        complete_initialization();
    } catch (const tf2::TransformException& ex) {
        spdlog::warn("TF lookup failed (will retry): {}", ex.what());
    }
}

void UKFNode::complete_initialization() {
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
    set_subscribers_and_publishers();

    time_step_ = std::chrono::milliseconds(
        this->get_parameter("publish_rate_ms").as_int());
    if (time_step_.count() <= 0)
        throw std::runtime_error("publish_rate_ms must be positive");
    odom_pub_timer_ =
        this->create_wall_timer(time_step_, [this]() { publish_odom(); });

    spdlog::info("UKF initialized");

    if (publish_nis_) {
        spdlog::info(
            "NIS publishing enabled on ukf/nis_dvl, ukf/nis_depth and "
            "ukf/nis_magnetometer");
    }
}

void UKFNode::reset_estimate() {
    estimate_ = prior_;
    first_imu_msg_received_ = propagated_ = dvl_received_ = depth_received_ =
        faulted_ = false;
    last_dvl_stamp_ = last_depth_stamp_ = -1;
}

void UKFNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
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
    const Eigen::Vector3d gyro = R_imu_ukf_ * raw_gyro;
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
                     "IMU gap exceeds limit; call ukf/reset to establish a "
                     "new navigation origin");
        return;
    }

    // move the imu measurement to base_link, removing the centripetal and
    // euler acceleration of the lever arm
    const Eigen::Vector3d omega = gyro - estimate_.mean.gyro_bias;
    const Eigen::Vector3d alpha = (gyro - previous_gyro_) / dt;
    ImuInput input;
    input.gyro = gyro;
    input.accel = R_imu_ukf_ * raw_accel -
                  omega.cross(omega.cross(T_imu_ukf_)) -
                  alpha.cross(T_imu_ukf_);
    input.dt = dt;

    const StateGaussian predicted = ukf_->predict(estimate_, input);
    if (!predicted.mean.as_vector().allFinite() ||
        !predicted.covariance.allFinite()) {
        faulted_ = true;
        RCLCPP_ERROR(get_logger(),
                     "Non-finite prediction; call ukf/reset to recover");
        return;
    }
    estimate_ = predicted;
    latest_input_ = input;

    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
        gyro_cov(msg->angular_velocity_covariance.data());
    if (gyro_cov.allFinite() &&
        gyro_cov.isApprox(gyro_cov.transpose(), 1e-10) &&
        gyro_cov.diagonal().minCoeff() > 0 &&
        gyro_cov.llt().info() == Eigen::Success)
        gyro_covariance_ = R_imu_ukf_ * gyro_cov * R_imu_ukf_.transpose();
    else
        gyro_covariance_ = ins_config_.noise_psd.block<3, 3>(NoiseIndex::gyro,
                                                             NoiseIndex::gyro) /
                           dt;
    previous_gyro_ = latest_gyro_measurement_ = gyro;
    last_imu_time_ = stamp;
    propagated_ = true;
}

bool UKFNode::accept_aiding_stamp(const rclcpp::Time& stamp,
                                  double& previous_stamp) {
    if (!propagated_ || faulted_)
        return false;
    const double age = (now() - stamp).seconds();
    const double skew = (stamp - last_imu_time_).seconds();
    // no rewind, only accept measurements close to the current epoch
    if (age < 0 || age > max_estimate_age_ ||
        std::abs(skew) > max_aiding_skew_ || stamp.seconds() <= previous_stamp)
        return false;
    previous_stamp = stamp.seconds();
    return true;
}

template <int M>
bool UKFNode::apply_update(const UpdateOutput<M>& update,
                           double threshold,
                           double& nis) {
    nis = std::numeric_limits<double>::quiet_NaN();
    const auto& innovation = update.innovation.mean;
    const auto& S = update.innovation.covariance;
    if (!innovation.allFinite() || !S.allFinite() ||
        !update.posterior.mean.as_vector().allFinite() ||
        !update.posterior.covariance.allFinite())
        return false;
    const Eigen::LLT<Eigen::Matrix<double, M, M>> solver(S);
    if (solver.info() != Eigen::Success)
        return false;
    nis = innovation.dot(solver.solve(innovation));
    if (!std::isfinite(nis) || nis < 0 || nis > threshold)
        return false;
    estimate_ = update.posterior;
    return true;
}

void UKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
    if (!accept_aiding_stamp(
            rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()),
            last_dvl_stamp_))
        return;

    Eigen::Vector3d measurement(msg->twist.twist.linear.x,
                                msg->twist.twist.linear.y,
                                msg->twist.twist.linear.z);
    Eigen::Matrix3d measurement_noise;
    if (dvl_use_msg_noise_) {
        measurement_noise << msg->twist.covariance[0], msg->twist.covariance[1],
            msg->twist.covariance[2], msg->twist.covariance[6],
            msg->twist.covariance[7], msg->twist.covariance[8],
            msg->twist.covariance[12], msg->twist.covariance[13],
            msg->twist.covariance[14];
    } else {
        measurement_noise =
            dvl_measurement_noise_std_.array().square().matrix().asDiagonal();
    }

    // correct rotation and translation: v_base = v_sensor - omega x T
    const Eigen::Vector3d omega_corrected =
        latest_gyro_measurement_ - estimate_.mean.gyro_bias;
    measurement = R_dvl_ukf_ * measurement - omega_corrected.cross(T_dvl_ukf_);
    measurement_noise = R_dvl_ukf_ * measurement_noise * R_dvl_ukf_.transpose();

    if (publish_debug_) {
        geometry_msgs::msg::TwistWithCovarianceStamped dvl_body_msg;
        dvl_body_msg.header.stamp = msg->header.stamp;
        dvl_body_msg.header.frame_id = frame("base_link");
        dvl_body_msg.twist.twist.linear.x = measurement.x();
        dvl_body_msg.twist.twist.linear.y = measurement.y();
        dvl_body_msg.twist.twist.linear.z = measurement.z();
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
            dvl_body_msg.twist.covariance.data())
            .topLeftCorner<3, 3>() = measurement_noise;
        dvl_body_pub_->publish(dvl_body_msg);
    }

    double nis = std::numeric_limits<double>::quiet_NaN();
    bool accepted = false;
    if (measurement.allFinite() && measurement_noise.allFinite() &&
        measurement_noise.isApprox(measurement_noise.transpose(), 1e-10) &&
        Eigen::LLT<Eigen::Matrix3d>(measurement_noise).info() ==
            Eigen::Success) {
        // the noise can come from the message, so the model is built per
        // message
        const DVLMeasurementModel dvl_model(
            DVLConfig{.velocity_noise = measurement_noise});
        accepted = apply_update(
            ukf_->update(estimate_, measurement, latest_input_, dvl_model),
            dvl_nis_threshold_, nis);
    }
    dvl_received_ = dvl_received_ || accepted;
    if (!accepted)
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "DVL correction rejected (invalid covariance/data or innovation)");

    if (publish_nis_) {
        std_msgs::msg::Float64 nis_msg;
        nis_msg.data = nis;
        nis_dvl_pub_->publish(nis_msg);
    }
}

void UKFNode::pressure_callback(
    const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
    if (!accept_aiding_stamp(
            rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()),
            last_depth_stamp_))
        return;
    if (!std::isfinite(msg->fluid_pressure) || !std::isfinite(msg->variance) ||
        msg->variance < 0)
        return;

    const double p_gauge = pressure_is_gauge_
                               ? msg->fluid_pressure
                               : msg->fluid_pressure - atmospheric_pressure_;
    const double depth = p_gauge / (water_density_ * gravity_);

    const double pressure_variance =
        pressure_use_msg_noise_ && msg->variance > 0.0
            ? msg->variance
            : pressure_measurement_noise_;
    const double depth_variance =
        pressure_variance / std::pow(water_density_ * gravity_, 2);

    if (publish_debug_) {
        std_msgs::msg::Float64 depth_msg;
        depth_msg.data = depth;
        depth_pub_->publish(depth_msg);
    }

    double nis = std::numeric_limits<double>::quiet_NaN();
    bool accepted = false;
    if (std::isfinite(depth) && std::isfinite(depth_variance) &&
        depth_variance > 0) {
        const DepthMeasurementModel depth_model(DepthConfig{
            .depth_noise_variance = depth_variance, .lever_arm = T_depth_ukf_});
        accepted = apply_update(
            ukf_->update(estimate_, Eigen::Matrix<double, 1, 1>(depth),
                         latest_input_, depth_model),
            depth_nis_threshold_, nis);
    }
    depth_received_ = depth_received_ || accepted;
    if (!accepted)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Depth correction rejected (invalid "
                             "covariance/data or innovation)");

    if (publish_nis_) {
        std_msgs::msg::Float64 nis_msg;
        nis_msg.data = nis;
        nis_depth_pub_->publish(nis_msg);
    }
}

void UKFNode::magnetometer_callback(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
    if (!accept_aiding_stamp(
            rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()),
            last_magnetometer_stamp_))
        return;

    // the message is in tesla, like the model, and in the sensor frame
    const Eigen::Vector3d field =
        R_magnetometer_ukf_ * Eigen::Vector3d(msg->magnetic_field.x,
                                              msg->magnetic_field.y,
                                              msg->magnetic_field.z);
    if (!field.allFinite())
        return;

    double nis = std::numeric_limits<double>::quiet_NaN();
    const bool accepted = apply_update(
        ukf_->update(estimate_, field, latest_input_, *magnetometer_model_),
        magnetometer_nis_threshold_, nis);
    if (!accepted)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Magnetometer correction rejected (invalid "
                             "covariance/data or innovation)");

    if (publish_nis_) {
        std_msgs::msg::Float64 nis_msg;
        nis_msg.data = nis;
        nis_magnetometer_pub_->publish(nis_msg);
    }
}

void UKFNode::publish_odom() {
    std_msgs::msg::Bool validity;
    const double age =
        first_imu_msg_received_ ? (now() - last_imu_time_).seconds() : -1;
    validity.data = propagated_ && dvl_received_ && depth_received_ &&
                    !faulted_ && age >= 0 && age <= max_estimate_age_;
    validity_pub_->publish(validity);
    if (!validity.data)
        return;

    nav_msgs::msg::Odometry odom_msg;
    const State& state = estimate_.mean;

    odom_msg.pose.pose.position.x = state.pos.x();
    odom_msg.pose.pose.position.y = state.pos.y();
    odom_msg.pose.pose.position.z = state.pos.z();

    odom_msg.pose.pose.orientation.w = state.quat.w();
    odom_msg.pose.pose.orientation.x = state.quat.x();
    odom_msg.pose.pose.orientation.y = state.quat.y();
    odom_msg.pose.pose.orientation.z = state.quat.z();

    // publishing the velocity in the body frame
    const Eigen::Vector3d v_body = state.quat.conjugate() * state.vel;
    odom_msg.twist.twist.linear.x = v_body.x();
    odom_msg.twist.twist.linear.y = v_body.y();
    odom_msg.twist.twist.linear.z = v_body.z();

    // publish bias-corrected body angular velocity
    const Eigen::Vector3d body_angular_vel =
        latest_gyro_measurement_ - state.gyro_bias;
    odom_msg.twist.twist.angular.x = body_angular_vel.x();
    odom_msg.twist.twist.angular.y = body_angular_vel.y();
    odom_msg.twist.twist.angular.z = body_angular_vel.z();

    const rclcpp::Time current_time = last_imu_time_;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = frame("odom");
    odom_msg.child_frame_id = frame("base_link");

    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        odom_msg.pose.covariance.data()) =
        ukf_output::pose_covariance(state, estimate_.covariance);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
        odom_msg.twist.covariance.data()) =
        ukf_output::twist_covariance(state, estimate_.covariance,
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
        publish_tf(state, current_time);
    }

    if (publish_biases_) {
        geometry_msgs::msg::Vector3Stamped accel_bias_msg;
        accel_bias_msg.header.stamp = current_time;
        // biases are in the body frame
        accel_bias_msg.header.frame_id = frame("base_link");
        accel_bias_msg.vector.x = state.accel_bias.x();
        accel_bias_msg.vector.y = state.accel_bias.y();
        accel_bias_msg.vector.z = state.accel_bias.z();
        accel_bias_pub_->publish(accel_bias_msg);

        geometry_msgs::msg::Vector3Stamped gyro_bias_msg;
        gyro_bias_msg.header = accel_bias_msg.header;
        gyro_bias_msg.vector.x = state.gyro_bias.x();
        gyro_bias_msg.vector.y = state.gyro_bias.y();
        gyro_bias_msg.vector.z = state.gyro_bias.z();
        gyro_bias_pub_->publish(gyro_bias_msg);
    }
}

void UKFNode::publish_tf(const State& state, const rclcpp::Time& time) {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = time;
    tf_msg.header.frame_id = frame("odom");
    tf_msg.child_frame_id = frame("base_link");

    tf_msg.transform.translation.x = state.pos.x();
    tf_msg.transform.translation.y = state.pos.y();
    tf_msg.transform.translation.z = state.pos.z();

    tf_msg.transform.rotation.w = state.quat.w();
    tf_msg.transform.rotation.x = state.quat.x();
    tf_msg.transform.rotation.y = state.quat.y();
    tf_msg.transform.rotation.z = state.quat.z();

    tf_broadcaster_->sendTransform(tf_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(UKFNode)

#include "gtsam_navigation/estimator.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gtsam_navigation {
/** Thin ROS boundary. The default mutually exclusive callback group serializes
 * subscriptions and timers even when loaded in a multithreaded container.
 */
class NavigationNode : public rclcpp::Node {
   public:
    explicit NavigationNode(const rclcpp::NodeOptions& options)
        : Node("gtsam_navigation", options) {
        const auto prefix =
            declare_parameter<std::string>("frame_prefix", "nautilus");
        const auto frame = [&prefix](const std::string& suffix) {
            return prefix.empty() ? suffix : prefix + "/" + suffix;
        };
        body_frame_ =
            declare_parameter<std::string>("body_frame", frame("base_link"));
        imu_frame_ =
            declare_parameter<std::string>("imu_frame", frame("imu_link"));
        dvl_frame_ =
            declare_parameter<std::string>("dvl_frame", frame("dvl_link"));
        odom_frame_ =
            declare_parameter<std::string>("odom_frame", frame("odom"));
        if (body_frame_.empty() || imu_frame_.empty() || dvl_frame_.empty() ||
            odom_frame_.empty()) {
            throw std::invalid_argument("Frame IDs must not be empty");
        }
        declare_config();
        const auto imu_topic =
            declare_parameter<std::string>("imu_topic", "imu/data_raw");
        const auto dvl_topic =
            declare_parameter<std::string>("dvl_topic", "dvl/twist");
        const auto odom_topic =
            declare_parameter<std::string>("odom_topic", "gtsam/odom");
        const auto status_topic =
            declare_parameter<std::string>("status_topic", "gtsam/status");
        hardware_id_ = declare_parameter<std::string>(
            "diagnostic_hardware_id", "STIM300 + Nucleus bottom track");
        publish_tf_ = declare_parameter<bool>("publish_tf", false);
        imu_timeout_ = declare_parameter<double>("imu_timeout", 0.2);
        dvl_timeout_ = declare_parameter<double>("dvl_timeout", 1.0);
        const double publish_rate =
            declare_parameter<double>("publish_rate", 125.0);
        for (double value : {imu_timeout_, dvl_timeout_, publish_rate}) {
            if (!std::isfinite(value) || value <= 0) {
                throw std::invalid_argument(
                    "Timeouts and publish_rate must be positive");
            }
        }
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS());
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            status_topic, rclcpp::QoS(1).reliable().transient_local());
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS().keep_last(2000),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                on_imu(*msg);
            });
        dvl_sub_ =
            create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                dvl_topic, rclcpp::SensorDataQoS().keep_last(100),
                [this](geometry_msgs::msg::TwistWithCovarianceStamped::
                           ConstSharedPtr msg) { on_dvl(*msg); });
        publish_timer_ =
            create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                              [this]() { publish(); });
        status_timer_ = create_wall_timer(std::chrono::seconds(1),
                                          [this]() { publish_status(); });
        tf_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                      [this]() { load_transforms(); });
        RCLCPP_INFO(get_logger(),
                    "Waiting for fixed Nautilus sensor transforms; bias states "
                    "remain internal");
    }

   private:
    void declare_config() {
        const auto profile = declare_parameter<std::string>(
            "imu_profile", "stim300_10g_provisional");
        if (profile == "stim300_30g") {
            config_.accel_noise_density = 0.21 / 60.0;
        } else if (profile != "stim300_10g_provisional" &&
                   profile != "stim300_10g") {
            throw std::invalid_argument("Unknown STIM300 profile");
        }
        config_.gravity = declare_parameter("gravity", config_.gravity);
        config_.accel_noise_density = declare_parameter(
            "accel_noise_density", config_.accel_noise_density);
        config_.gyro_noise_density =
            declare_parameter("gyro_noise_density", config_.gyro_noise_density);
        config_.accel_bias_random_walk = declare_parameter(
            "accel_bias_random_walk", config_.accel_bias_random_walk);
        config_.gyro_bias_random_walk = declare_parameter(
            "gyro_bias_random_walk", config_.gyro_bias_random_walk);
        config_.integration_sigma =
            declare_parameter("integration_sigma", config_.integration_sigma);
        config_.lag = declare_parameter("lag_seconds", config_.lag);
        config_.keyframe_interval =
            declare_parameter("keyframe_interval", config_.keyframe_interval);
        config_.reorder_delay =
            declare_parameter("reorder_delay", config_.reorder_delay);
        config_.max_imu_gap =
            declare_parameter("max_imu_gap", config_.max_imu_gap);
        config_.initialization_duration = declare_parameter(
            "initialization_duration", config_.initialization_duration);
        config_.stationary_accel_std = declare_parameter(
            "stationary_accel_std", config_.stationary_accel_std);
        config_.stationary_gyro_std = declare_parameter(
            "stationary_gyro_std", config_.stationary_gyro_std);
        config_.stationary_gyro_norm = declare_parameter(
            "stationary_gyro_norm", config_.stationary_gyro_norm);
        config_.stationary_gravity_tolerance =
            declare_parameter("stationary_gravity_tolerance",
                              config_.stationary_gravity_tolerance);
        config_.dvl_gate_squared =
            declare_parameter("dvl_gate_squared", config_.dvl_gate_squared);
        const int capacity =
            declare_parameter<int>("max_buffer_samples", 20000);
        if (capacity < 10) {
            throw std::invalid_argument(
                "max_buffer_samples must be at least 10");
        }
        config_.max_buffer_samples = static_cast<std::size_t>(capacity);
        // Validate numerical parameters immediately, before waiting for TF.
        Estimator validate(config_);
    }

    static gtsam::Pose3 pose_from_transform(
        const geometry_msgs::msg::Transform& transform) {
        const auto& q = transform.rotation;
        Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        const auto& t = transform.translation;
        if (!rotation.coeffs().allFinite() ||
            std::abs(rotation.norm() - 1.0) > 1e-3 ||
            !gtsam::Vector3(t.x, t.y, t.z).allFinite()) {
            throw std::invalid_argument("Invalid fixed sensor transform");
        }
        return gtsam::Pose3(gtsam::Rot3(rotation.normalized()),
                            gtsam::Point3(t.x, t.y, t.z));
    }

    void load_transforms() {
        if (estimator_) {
            return;
        }
        try {
            body_p_imu_ = pose_from_transform(
                tf_buffer_
                    ->lookupTransform(body_frame_, imu_frame_,
                                      tf2::TimePointZero)
                    .transform);
            config_.imu_p_dvl = pose_from_transform(
                tf_buffer_
                    ->lookupTransform(imu_frame_, dvl_frame_,
                                      tf2::TimePointZero)
                    .transform);
            estimator_ = std::make_unique<Estimator>(config_);
            tf_timer_->cancel();
            RCLCPP_INFO(get_logger(),
                        "Fixed sensor transforms loaded; collecting stationary "
                        "IMU samples");

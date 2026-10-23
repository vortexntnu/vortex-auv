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
        } catch (const tf2::TransformException& error) {
            last_error_ = error.what();
        } catch (const std::exception& error) {
            last_error_ = error.what();
        }
    }

    double measurement_time(const builtin_interfaces::msg::Time& stamp) {
        const auto ns = rclcpp::Time(stamp).nanoseconds();
        if (!epoch_ns_) {
            // Keep floating point integration times small without losing ROS
            // timestamps.
            epoch_ns_ = (static_cast<int64_t>(stamp.sec) - 1) * 1000000000LL;
        }
        return static_cast<double>(ns - *epoch_ns_) * 1e-9;
    }

    void on_imu(const sensor_msgs::msg::Imu& msg) {
        if (!estimator_) {
            return;
        }
        if (msg.header.frame_id != imu_frame_ ||
            msg.angular_velocity_covariance[0] < 0 ||
            msg.linear_acceleration_covariance[0] < 0) {
            ++rejected_ros_;
            last_error_ = "IMU frame mismatch or unavailable acceleration/gyro";
            return;
        }
        const auto& a = msg.linear_acceleration;
        const auto& w = msg.angular_velocity;
        try {
            if (estimator_->add_imu({measurement_time(msg.header.stamp),
                                     {a.x, a.y, a.z},
                                     {w.x, w.y, w.z}})) {
                latest_imu_stamp_ns_ =
                    std::max(latest_imu_stamp_ns_,
                             rclcpp::Time(msg.header.stamp).nanoseconds());
            }
        } catch (const std::exception& error) {
            ++rejected_ros_;
            last_error_ = error.what();
        }
    }

    void on_dvl(const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
        if (!estimator_ || !epoch_ns_) {
            return;
        }
        if (msg.header.frame_id != dvl_frame_) {
            ++rejected_ros_;
            last_error_ = "DVL must be expressed at the DVL origin in DVL axes";
            return;
        }
        gtsam::Matrix3 covariance;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                covariance(row, col) = msg.twist.covariance[row * 6 + col];
            }
        }
        const auto& v = msg.twist.twist.linear;
        try {
            estimator_->add_dvl({measurement_time(msg.header.stamp),
                                 {v.x, v.y, v.z},
                                 covariance});
        } catch (const std::exception& error) {
            ++rejected_ros_;
            last_error_ = error.what();
        }
    }

    void publish() {
        if (!estimator_ || !epoch_ns_ || prediction_failed_) {
            return;
        }
        const double age =
            static_cast<double>(now().nanoseconds() - latest_imu_stamp_ns_) *
            1e-9;
        if (age > imu_timeout_ || age < -imu_timeout_) {
            return;
        }
        try {
            const auto estimate = estimator_->latest();
            if (!estimate || estimate->time <= last_published_time_) {
                return;
            }
            nav_msgs::msg::Odometry msg;
            msg.header.stamp = rclcpp::Time(
                *epoch_ns_ +
                static_cast<int64_t>(std::llround(estimate->time * 1e9)));
            msg.header.frame_id = odom_frame_;
            msg.child_frame_id = imu_frame_;
            const auto& p = estimate->pose.translation();
            const auto q = estimate->pose.rotation().toQuaternion();
            msg.pose.pose.position.x = p.x();
            msg.pose.pose.position.y = p.y();
            msg.pose.pose.position.z = p.z();
            msg.pose.pose.orientation.w = q.w();
            msg.pose.pose.orientation.x = q.x();
            msg.pose.pose.orientation.y = q.y();
            msg.pose.pose.orientation.z = q.z();
            const auto v =
                estimate->pose.rotation().unrotate(estimate->velocity);
            msg.twist.twist.linear.x = v.x();
            msg.twist.twist.linear.y = v.y();
            msg.twist.twist.linear.z = v.z();
            msg.twist.twist.angular.x = estimate->angular_velocity.x();
            msg.twist.twist.angular.y = estimate->angular_velocity.y();
            msg.twist.twist.angular.z = estimate->angular_velocity.z();
            const auto [pose_cov, twist_cov] =
                odometry_covariances(*estimate, config_);
            for (int row = 0; row < 6; ++row) {
                for (int col = 0; col < 6; ++col) {
                    msg.pose.covariance[row * 6 + col] = pose_cov(row, col);
                    msg.twist.covariance[row * 6 + col] = twist_cov(row, col);
                }
            }
            odom_pub_->publish(msg);
            last_published_time_ = estimate->time;
            if (publish_tf_) {
                // The URDF already owns base_link -> imu_link. Publish the
                // equivalent odom -> base_link edge so the IMU never acquires a
                // second TF parent.
                const auto body_pose =
                    estimate->pose.compose(body_p_imu_.inverse());
                const auto body_q = body_pose.rotation().toQuaternion();
                geometry_msgs::msg::TransformStamped transform;
                transform.header = msg.header;
                transform.child_frame_id = body_frame_;
                transform.transform.translation.x = body_pose.x();
                transform.transform.translation.y = body_pose.y();
                transform.transform.translation.z = body_pose.z();
                transform.transform.rotation.w = body_q.w();
                transform.transform.rotation.x = body_q.x();
                transform.transform.rotation.y = body_q.y();
                transform.transform.rotation.z = body_q.z();
                tf_broadcaster_->sendTransform(transform);
            }
        } catch (const std::exception& error) {
            prediction_failed_ = true;
            last_error_ = std::string("prediction failed; restart required: ") +
                          error.what();
            RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
        }
    }

    void publish_status() {
        using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp = now();
        Diagnostic diagnostic;
        diagnostic.name =
            std::string(get_fully_qualified_name()) + "/navigation";
        diagnostic.hardware_id = hardware_id_;
        diagnostic.level = Diagnostic::WARN;
        diagnostic.message = "waiting for fixed sensor transforms";
        const auto add = [&diagnostic](const std::string& key,
                                       const std::string& value) {
            diagnostic_msgs::msg::KeyValue item;
            item.key = key;
            item.value = value;
            diagnostic.values.push_back(item);
        };
        if (estimator_) {
            const auto& status = estimator_->status();
            diagnostic.message = status.detail;
            diagnostic.level =
                status.fault
                    ? Diagnostic::ERROR
                    : (status.initialized ? Diagnostic::OK : Diagnostic::WARN);
            const double imu_age = static_cast<double>(now().nanoseconds() -
                                                       latest_imu_stamp_ns_) *
                                   1e-9;
            if (!status.fault &&
                (imu_age > imu_timeout_ || imu_age < -imu_timeout_)) {
                diagnostic.level = Diagnostic::ERROR;
                diagnostic.message =
                    "IMU stale or clock mismatch; odometry publication stopped";
            } else if (!status.fault && status.initialized && epoch_ns_ &&
                       (status.last_dvl_time < 0 ||
                        static_cast<double>(now().nanoseconds() - *epoch_ns_) *
                                    1e-9 -
                                status.last_dvl_time >
                            dvl_timeout_)) {
                diagnostic.level = Diagnostic::WARN;
                diagnostic.message = "DVL unavailable; IMU dead reckoning";
            }
            add("initialized", status.initialized ? "true" : "false");
            add("rejected_imu", std::to_string(status.rejected_imu));
            add("rejected_dvl", std::to_string(status.rejected_dvl));
            add("active_states", std::to_string(status.active_states));
            add("factor_slots", std::to_string(status.factor_slots));
            add("buffered_samples", std::to_string(status.buffered_samples));
            add("imu_age_seconds", std::to_string(imu_age));
        }
        if (prediction_failed_) {
            diagnostic.level = Diagnostic::ERROR;
            diagnostic.message = last_error_;
        }
        add("rejected_ros_messages", std::to_string(rejected_ros_));
        add("last_input_error", last_error_);
        msg.status.push_back(diagnostic);
        status_pub_->publish(msg);
    }

    Config config_;
    gtsam::Pose3 body_p_imu_;
    std::unique_ptr<Estimator> estimator_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string body_frame_, imu_frame_, dvl_frame_, odom_frame_, last_error_;
    std::string hardware_id_;
    std::optional<int64_t> epoch_ns_;
    int64_t latest_imu_stamp_ns_ = 0;
    double last_published_time_ = -1, imu_timeout_ = 0.2, dvl_timeout_ = 1.0;
    bool publish_tf_ = false, prediction_failed_ = false;
    std::size_t rejected_ros_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        status_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_, status_timer_, tf_timer_;
};
}  // namespace gtsam_navigation
RCLCPP_COMPONENTS_REGISTER_NODE(gtsam_navigation::NavigationNode)

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

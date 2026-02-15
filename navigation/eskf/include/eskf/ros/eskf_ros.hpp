#ifndef ESKF__ESKF_ROS_HPP_
#define ESKF__ESKF_ROS_HPP_

#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "eskf/lib/eskf.hpp"
#include "eskf/lib/typedefs.hpp"
#include "spdlog/spdlog.h"

#ifdef ESKF_HAS_LANDMARK_EGOMOTION
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vortex_msgs/msg/landmark_array.hpp>
#include "landmark_egomotion/lib/landmark_egomotion.hpp"
#include "landmark_egomotion/lib/vo_typedefs.hpp"
#endif

class ESKFNode : public rclcpp::Node {
   public:
    explicit ESKFNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    // @brief Callback function for the imu topic
    // @param msg: Imu message containing the imu data
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // @brief Callback function for the dvl topic
    // @param msg: TwistWithCovarianceStamped message containing the dvl data
    void dvl_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // @brief Publish the odometry message
    void publish_odom();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

#ifdef ESKF_HAS_LANDMARK_EGOMOTION
    void setup_vo(const EskfParams& eskf_params);

    void landmark_callback(
        const vortex_msgs::msg::LandmarkArray::SharedPtr msg);

    LandmarkESKF* landmark_eskf_{nullptr};

    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        landmark_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string vo_base_frame_;
    std::string vo_cam_frame_;
    bool have_last_marker_{false};
    uint16_t last_marker_id_{0};
#endif

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_pub_;

    std::chrono::milliseconds time_step;

    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    std::unique_ptr<ESKF> eskf_;

    bool first_imu_msg_received_ = false;

    Eigen::Matrix3d R_imu_eskf_{};

    Eigen::Matrix3d R_dvl_eskf_{};

    rclcpp::Time last_imu_time_{};

    // Latest gyro measurement (used for publishing odom output of eskf)
    Eigen::Vector3d latest_gyro_measurement_{};
};

#endif  // ESKF__ESKF_ROS_HPP_

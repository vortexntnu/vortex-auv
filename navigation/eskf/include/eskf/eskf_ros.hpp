#ifndef ESKF__ESKF_ROS_HPP_
#define ESKF__ESKF_ROS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <stonefish_ros2/msg/dvl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "eskf/eskf.hpp"
#include "eskf/typedefs.hpp"
#include "spdlog/spdlog.h"

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
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr msg);

    // @brief Publish the odometry message
    void publish_odom();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

    // @brief lookup transforms
    void initialize_static_transforms();

    // @brief broadcast the State as a TF
    void publish_tf(const StateQuat& nom_state);

    // Subscribers and Publishers

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_pub_;

    // Member variable for the ESKF instance

    std::chrono::milliseconds time_step;

    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    std::unique_ptr<ESKF> eskf_;

    bool first_imu_msg_received_ = false;

    Eigen::Matrix3d R_imu_eskf_{};
    Eigen::Vector3d T_imu_eskf_{};

    Eigen::Matrix3d R_dvl_eskf_{};
    Eigen::Vector3d T_dvl_eskf_{};

    Eigen::Vector3d T_depth_eskf_{};

    rclcpp::Time last_imu_time_{};

    // Latest gyro measurement (used for publishing odom output of eskf)
    Eigen::Vector3d latest_gyro_measurement_{};

    // TF2 Handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Flags and Storage
    bool use_tf_transforms_ = false;
    bool tf_sensors_loaded_ = false;

    // hold the transfer from Sensor -> Base Link
    Eigen::Isometry3d Tf_base_imu_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tf_base_dvl_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tf_base_depth_ = Eigen::Isometry3d::Identity();
};

#endif  // ESKF__ESKF_ROS_HPP_

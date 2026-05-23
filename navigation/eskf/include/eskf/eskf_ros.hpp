#ifndef ESKF__ESKF_ROS_HPP_
#define ESKF__ESKF_ROS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include "eskf/eskf.hpp"
#include "eskf/typedefs.hpp"

class ESKFNode : public rclcpp::Node {
   public:
    explicit ESKFNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    // @brief Callback function for the imu topic
    // @param msg: Imu message containing the imu data
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    // @brief Callback function for the dvl topic
    // @param msg: TwistWithCovarianceStamped message containing the dvl data
    void dvl_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
            msg);

    void pressure_callback(
        const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg);

    // @brief Publish the odometry message
    void publish_odom();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

    // @brief lookup transforms
    void lookup_static_transforms();

    // @brief Create subs/pubs and start the publish timer. Called once all
    // required TF transforms are available (or immediately if none use TF).
    void complete_initialization();

    // @brief broadcast the State as a TF
    void publish_tf(const NominalState& nom_state,
                    const rclcpp::Time& current_time);

    // Subscribers and Publishers

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr depth_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_dvl_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_depth_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        dvl_body_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        accel_bias_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        gyro_bias_pub_;

    // Member variable for the ESKF instance

    std::chrono::milliseconds time_step_{1};

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

    std::string frame(const std::string& name) const {
        return frame_prefix_.empty() ? name : frame_prefix_ + "/" + name;
    }

    bool publish_debug_{false};
    bool publish_nis_{false};

    // Flags and Storage
    std::string frame_prefix_{""};
    bool publish_tf_{false};
    bool publish_pose_{false};
    bool publish_twist_{false};
    bool publish_biases_{false};

    bool imu_use_tf_transform_{true};
    Eigen::Isometry3d Tf_base_imu_ = Eigen::Isometry3d::Identity();

    bool dvl_use_tf_transform_{true};
    bool dvl_use_msg_noise_{true};
    Eigen::Vector3d dvl_measurement_noise_std_{0.1, 0.1, 0.1};
    Eigen::Isometry3d Tf_base_dvl_ = Eigen::Isometry3d::Identity();

    bool pressure_use_tf_transform_{true};
    bool pressure_use_msg_noise_{false};
    double pressure_measurement_noise_{40000.0};
    Eigen::Isometry3d Tf_base_depth_ = Eigen::Isometry3d::Identity();

    double gravity_;
    double water_density_;
    double atmospheric_pressure_;
    bool pressure_is_gauge_{false};
};

#endif  // ESKF__ESKF_ROS_HPP_

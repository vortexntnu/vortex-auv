#ifndef ESKF_ROS_HPP
#define ESKF_ROS_HPP

#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "eskf/eskf.hpp"
#include "eskf/typedefs.hpp"
#include "spdlog/spdlog.h"

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <Eigen/Dense>

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

    void visualEgomotion_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // @brief Publish the odometry message
    void publish_odom();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Set the parameters for the eskf
    void set_parameters();

    void setup_services();
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr visualEgomotion_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr nis_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_imu_dt_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vo_pos_norm_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vo_ang_norm_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vo_nis_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_vo_rejects_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_vo_anchor_valid_;

    std::chrono::milliseconds time_step;

    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    std::unique_ptr<ESKF> eskf_;

    Eigen::Matrix3d R_imu_eskf_{};

    rclcpp::Time last_imu_time_{};

    bool first_imu_msg_received_ = false;

    static inline void pub_f64(
      const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub,
      double v) {
        if (!pub) return;
        std_msgs::msg::Float64 m;
        m.data = v;
        pub->publish(m);
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_gating_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_anchor_srv_;


};

#endif  // ESKF_ROS_HPP

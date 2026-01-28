#ifndef PID_CONTROLLER_DP_EULER__PID_CONTROLLER_ROS_HPP_
#define PID_CONTROLLER_DP_EULER__PID_CONTROLLER_ROS_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pid_controller_dp_euler/pid_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vortex_msgs/msg/operation_mode.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include "pid_controller_dp_euler/typedefs.hpp"

class PIDControllerNode : public rclcpp::Node {
   public:
    PIDControllerNode();

   private:
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void software_mode_callback(
        const vortex_msgs::msg::OperationMode::SharedPtr msg);

    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void twist_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    void guidance_callback(
        const vortex_msgs::msg::ReferenceFilter::SharedPtr msg);

    void publish_tau();

    void set_pid_params();

    void set_subscribers_and_publisher();

    PIDController pid_controller_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killswitch_sub_;

    rclcpp::Subscription<vortex_msgs::msg::OperationMode>::SharedPtr
        software_mode_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    rclcpp::Subscription<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        guidance_sub_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_pub_;

    rclcpp::TimerBase::SharedPtr tau_pub_timer_;

    std::chrono::milliseconds time_step_;

    Eta eta_;

    Eta eta_d_;

    Nu nu_;

    Eta eta_dot_d_;

    bool killswitch_on_ = false;

    uint8_t software_mode_;
};

#endif  // PID_CONTROLLER_DP_EULER__PID_CONTROLLER_ROS_HPP_

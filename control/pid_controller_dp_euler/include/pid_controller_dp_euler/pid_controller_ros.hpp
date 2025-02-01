#ifndef PID_CONTROLLER_ROS_HPP
#define PID_CONTROLLER_ROS_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pid_controller_dp_euler/pid_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vortex_msgs/msg/reference_filter.hpp>
#include "pid_controller_dp_euler/typedefs.hpp"

class PIDControllerNode : public rclcpp::Node {
   public:
    explicit PIDControllerNode();

   private:
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void software_mode_callback(const std_msgs::msg::String::SharedPtr msg);

<<<<<<< HEAD
    void active_controller_callback(const std_msgs::msg::String::SharedPtr msg);

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
=======
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void twist_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
>>>>>>> origin/434-task-dp-controller

    void guidance_callback(
        const vortex_msgs::msg::ReferenceFilter::SharedPtr msg);

    void publish_tau();

    void set_pid_params();

    PIDController pid_controller_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killswitch_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr software_mode_sub_;

<<<<<<< HEAD
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_controller_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
=======
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
>>>>>>> origin/434-task-dp-controller

    rclcpp::Subscription<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        guidance_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr tau_pub_;

    rclcpp::TimerBase::SharedPtr tau_pub_timer_;

    std::chrono::milliseconds time_step_;

    Eta eta_;

    Eta eta_d_;

    Nu nu_;

    Eta eta_dot_d_;

    bool killswitch_on_;

    std::string software_mode_;

    std::string active_controller_;
};

#endif

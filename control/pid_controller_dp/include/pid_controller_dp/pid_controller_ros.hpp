#ifndef PID_CONTROLLER_ROS_HPP
#define PID_CONTROLLER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "pid_controller_dp/pid_controller.hpp"
#include <geometry_msgs/msg/wrench.hpp>


class PIDControllerNode : public rclcpp::Node
{
    public:
        explicit PIDControllerNode();

    private:
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void publish_tau();

        PIDController pid_controller_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr tau_pub_;

        rclcpp::TimerBase::SharedPtr tau_pub_timer_;

        std::chrono::milliseconds time_step_;
};

#endif
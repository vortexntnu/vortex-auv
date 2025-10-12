#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/velocity_controller.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class test_VC : public rclcpp::Node{
    public:
    test_VC();
    //Callback functions
    void read_thrust(geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void send_guidance();
    void send_state();
    
    //Variables
    //guidance_data reference;
    guidance_data current_state;
    //Subscribers and publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_guidance;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_twist;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_pose;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_thrust;
    //Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    //Messages
    std::vector<double> thrust_vector;
    std_msgs::msg::Float64MultiArray reference_msg;

    //Topics
    std::string topic_twist;
    std::string topic_pose;
    std::string topic_thrust;
    std::string topic_guidance;

    //MSGS
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
};

geometry_msgs::msg::Quaternion euler_angle_to_quaternion(double roll, double pitch, double yaw);
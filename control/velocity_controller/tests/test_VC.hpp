#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "vortex_msgs/msg/los_guidance.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "velocity_controller/utilities.hpp"

class test_VC : public rclcpp::Node{
    public:
    test_VC();
    //Callback functions
    void send_guidance();
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);
    
    //Variables
    
    //Subscribers and publishers 
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr publisher_guidance;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr publisher_state;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_state;
    //Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    //Messages
    vortex_msgs::msg::LOSGuidance reference_msg;

    //Topics
    std::string topic_guidance;
    std::string topic_state="/state";
    std::string topic_odometry;
    

    //MSGS
    double time1=0;
};

geometry_msgs::msg::Quaternion euler_angle_to_quaternion(double roll, double pitch, double yaw);
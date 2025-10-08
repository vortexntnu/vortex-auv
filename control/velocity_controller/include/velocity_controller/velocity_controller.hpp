#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "velocity_controller/PID_setup.hpp"


//#include "vortex-msgs/msg" kan legge til nye meldinger nå

class guidance_data{
    public:
    double surge;    double pitch;    double yaw;
    guidance_data(std_msgs::msg::Float64MultiArray msg);
    guidance_data():surge(0.0), pitch(0.0), yaw(0.0) {};
    
    guidance_data operator-(const guidance_data& other) const;
    guidance_data& operator=(const std_msgs::msg::Float64MultiArray& msg);
};

class Velocity_node : public rclcpp::Node{
    public:
    Velocity_node();

    //Timer functions
    void publish_thrust();
    void calc_thrust();

    //Callback functions
    void guidance_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg_ptr);
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg_ptr);
    void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg_ptr);
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_ptr);

    //Publisher instance
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_thrust;
    
    //Timer instance
    rclcpp::TimerBase::SharedPtr timer_calculation;
    rclcpp::TimerBase::SharedPtr timer_publish;
    
    //Subscriber instance
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscriber_twist;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_pose;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_guidance;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_killswitch;

    //Variables for topics

    std::string topic_thrust;
    std::string topic_guidance;
    std::string topic_killswitch;
    std::string topic_twist;
    std::string topic_pose;

    //Variables for timers
    int calculation_rate;
    int publish_rate;
    double max_force;

    //Stored wrenches values
    std_msgs::msg::Float64MultiArray reference_in;
    guidance_data reference;
    guidance_data current_state;
    geometry_msgs::msg::WrenchStamped thrust_out;

    //PID controllers
    PID_controller PID_surge;
    PID_controller PID_yaw;
    PID_controller PID_pitch;

    
};





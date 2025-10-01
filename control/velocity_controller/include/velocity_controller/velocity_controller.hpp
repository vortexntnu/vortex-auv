#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

//#include "vortex-msgs/msg" kan legge til nye meldinger nå

class Velocity_node : public rclcpp::Node{
    public:
    Velocity_node();
    //Timer functions
    void publish_thrust();
    //Callback functions
    void guidance_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    void killswitch_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    void twist_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    
    //Publisher instance
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_thrust;
    
    //Timer instance
    rclcpp::TimerBase::SharedPtr timer_;
    //Subscriber instance
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_twist;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_guidance;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_killswitch;

    //Variables for topics

    std::string topic_thrust;
    std::string topic_guidance;
    std::string topic_killswitch;
    std::string topic_twist;

    //Stored values
    geometry_msgs::msg::WrenchStamped reference;
    geometry_msgs::msg::WrenchStamped current_velocity_and_orientation;
};


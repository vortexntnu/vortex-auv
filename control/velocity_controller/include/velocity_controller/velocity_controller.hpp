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
    void calc_thrust();

    //Callback functions
    void guidance_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    void killswitch_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    void twist_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    
    //Publisher instance
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_thrust;
    
    //Timer instance
    rclcpp::TimerBase::SharedPtr timer_PID;
    rclcpp::TimerBase::SharedPtr timer_publish;
    
    //Subscriber instance
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_twist;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_guidance;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_killswitch;

    //Variables for topics

    std::string topic_thrust;
    std::string topic_guidance;
    std::string topic_killswitch;
    std::string topic_twist;

    //Variables for timers
    int calculation_rate;
    int publish_rate;
    double max_force;

    //Stored wrenches values
    geometry_msgs::msg::WrenchStamped reference;
    geometry_msgs::msg::WrenchStamped current_twist;
    geometry_msgs::msg::WrenchStamped thrust;

    //PID parameters temporary
    double k_p = 5.0;
    double k_i = 2.0;
    double k_d = 0.0;
    double integral = 0.0;
    double previous_error = 0.0; //improved Riemanns sums
};




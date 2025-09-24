#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
//#include "vortex-msgs/msg" kan legge til nye meldinger nå

class Velocity_node : public rclcpp::Node{
    public:
    Velocity_node();
    //publiserer fart
    void send_velocity();
    //New reference
    void recieve_new_reference(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    //New current velocity and orientation
    void recieve_new_velocity_and_orientation(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr);
    //Alle funksjonens variabler
//Publisher og timer instansene
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_velocity_and_orientation_;
    //Variabler

    std::string info_out_topic;
    std::string reference_topic;
    std::string velocity_and_orientation_topic;
    //lagrer referanse og nåværende fart

    geometry_msgs::msg::WrenchStamped reference;
    geometry_msgs::msg::WrenchStamped current_velocity_and_orientation;
};


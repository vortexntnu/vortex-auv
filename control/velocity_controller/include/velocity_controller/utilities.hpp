#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


class angle{
    public:
    double phit=0.0;
    double thetat=0.0;
    double psit=0.0;
};
angle quaternion_to_euler_angle(double w, double x, double y, double z);

class State{
    //Dataclass to store state values for LQR controller
    public:
    double surge=0.0;    double pitch=0.0;    double yaw=0.0;
    double integral_surge=0.0;    double integral_pitch=0.0;    double integral_yaw=0.0;
};

class Guidance_data:public State{
    public:
    //double surge;    double pitch;    double yaw;
    Guidance_data(std_msgs::msg::Float64MultiArray msg);
    Guidance_data(double surge, double pitch, double yaw):State{surge, pitch, yaw} {};
    Guidance_data():State{0, 0, 0} {};
    
    Guidance_data operator-(const Guidance_data& other) const;
    Guidance_data& operator=(const std_msgs::msg::Float64MultiArray& msg);
};

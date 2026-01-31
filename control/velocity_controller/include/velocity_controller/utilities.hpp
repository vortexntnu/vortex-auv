#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "vortex_msgs/msg/los_guidance.hpp"
#include <Eigen/Dense>


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
    double surge=0.0, sway=0.0, heave=0.0,  roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0; //roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0;
    double roll=0.0, pitch=0.0, yaw=0.0; //phi, theta, psi
    //double integral_surge=0.0;    double integral_pitch=0.0;    double integral_yaw=0.0;
    State(double surge=0,double pitch=0, double yaw=0):surge{surge}, pitch{pitch},yaw{yaw}{};
    State(){};
};

class Guidance_data:public State{
    public:
    //double surge;    double pitch;    double yaw;
    Guidance_data(vortex_msgs::msg::LOSGuidance msg):State{msg.surge,msg.pitch,msg.yaw}{};
    Guidance_data(double surge, double pitch, double yaw):State{surge, pitch, yaw} {};
    Guidance_data():State{0, 0, 0} {};
    
    Guidance_data operator-(const Guidance_data& other) const;
    Guidance_data& operator=(const std_msgs::msg::Float64MultiArray& msg);
};

angle NED_to_BODY(const angle &a,const State &s);
Eigen::Vector3d NED_to_BODY(const Eigen::Vector3d &a, const State &s);


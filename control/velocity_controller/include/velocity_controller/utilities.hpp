#pragma once
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "vortex_msgs/msg/los_guidance.hpp"
#include <Eigen/Dense>
#include <casadi/casadi.hpp>


struct angle{
    double phit=0.0;
    double thetat=0.0;
    double psit=0.0;
};
angle quaternion_to_euler_angle(double w, double x, double y, double z);
geometry_msgs::msg::Quaternion euler_angle_to_quaternion(double roll, double pitch, double yaw);

class State{
    //Dataclass to store state values for LQR controller
    public:
    double surge=0.0, sway=0.0, heave=0.0,  roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0; //roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0;
    double roll=0.0, pitch=0.0, yaw=0.0; //phi, theta, psi
    double w=0.0, x=0.0,y=0.0,z=0.0;
    //double integral_surge=0.0;    double integral_pitch=0.0;    double integral_yaw=0.0;
    State(double surge=0,double pitch=0, double yaw=0):surge{surge}, pitch{pitch},yaw{yaw}{};

    //State(){};
    State operator=(int n){if (n){surge=0.0,sway=0.0,heave=0.0,roll_rate=0.0,pitch_rate=0.0,yaw_rate=0.0,roll=0.0,pitch=0.0,yaw=0.0;} return *this;};
    State operator=(nav_msgs::msg::Odometry::SharedPtr rhs);
    angle get_angle();
};
//TODO: fix these so that changing the quaternions changes the angles, so that the state is always consistent
class Guidance_data{
    public:
    double surge=0.0;    double pitch=0.0;    double yaw=0.0;
    //Guidance_data(vortex_msgs::msg::LOSGuidance msg):State{msg.surge,msg.pitch,msg.yaw}{};
    Guidance_data(double surge, double pitch, double yaw):surge{surge},pitch{pitch}, yaw{yaw} {};
    Guidance_data():surge{0.0}, pitch{0.0}, yaw{0.0} {};
    //Guidance_data():State{0, 0, 0} {};
    
    //Guidance_data operator-(const Guidance_data& other) const;
    Guidance_data& operator=(const vortex_msgs::msg::LOSGuidance::SharedPtr& msg);
};

//angle NED_to_BODY(const angle &a,const State &s);
//Eigen::Vector3d NED_to_BODY(const Eigen::Vector3d &a, const State &s);
//Eigen::Matrix3d R_ned_to_body(double roll, double pitch, double yaw);
//Eigen::Matrix3d T_euler_to_body(double roll, double pitch);
Eigen::Vector3d body_rates_to_euler_rates(double roll, double pitch,double p, double q, double r);
angle angle_NED_to_body( double roll_des, double pitch_des, double yaw_des, double roll, double pitch, double yaw);
angle angle_NED_to_body(angle desired, angle state);
inline angle angle_NED_to_body(angle desired, angle state){return angle_NED_to_body(desired.phit, desired.thetat, desired.psit, state.phit, state.thetat, state.psit);}



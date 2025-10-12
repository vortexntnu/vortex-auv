#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class PID_controller {
    public:
    PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output);
    PID_controller(double k_p, double k_i, double k_d) : PID_controller(k_p, k_i, k_d, 100.0, -100.0) {};
    void calculate_thrust(double reference, double current_position, double dt);
    void reset_controller();
    double output();
    void set_output_limits(double min_output, double max_output);
    private:
    double k_p;
    double k_i;
    double k_d;
    double integral;
    double previous_error;
    double previous_position;
    double last_output;
    double max_output;
    double min_output;
};
class angle{
    public:
    double phit=0.0;
    double thetat=0.0;
    double psit=0.0;
};
angle quaternion_to_euler_angle(double w, double x, double y, double z);

class guidance_data{
    public:
    double surge;    double pitch;    double yaw;
    guidance_data(std_msgs::msg::Float64MultiArray msg);
    guidance_data(double surge, double pitch, double yaw):surge(surge), pitch(pitch), yaw(yaw) {};
    guidance_data():surge(0), pitch(0), yaw(0) {};
    
    guidance_data operator-(const guidance_data& other) const;
    guidance_data& operator=(const std_msgs::msg::Float64MultiArray& msg);
};

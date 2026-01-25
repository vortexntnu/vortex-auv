#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "utilities.hpp"

class PID_controller {
    public:
    PID_controller( double k_p=0, double k_i=0, double k_d=0, double max_output=100, double min_output=-100);
    //PID_controller(double k_p, double k_i, double k_d) : PID_controller(k_p, k_i, k_d, 100.0, -100.0) {};
    double calculate_thrust(double error, double dt);
    void reset_controller();
    double get_output();
    bool set_output_limits(double min_output, double max_output);
    void set_parameters(double k_p,double k_i, double k_d);
    private:
    double k_p;
    double k_i;
    double k_d;
    double integral=0;
    double previous_error=0;
    double output=0;
    double max_output;
    double min_output;
};


#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "utilities.hpp"

class PID_controller {
    public:
    PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output);
    PID_controller(double k_p, double k_i, double k_d) : PID_controller(k_p, k_i, k_d, 100.0, -100.0) {};
    void calculate_thrust(double error, double dt);
    void reset_controller();
    double output();
    void set_output_limits(double min_output, double max_output);
    private:
    double k_p;
    double k_i;
    double k_d;
    double integral;
    double previous_error;
    double last_output;
    double max_output;
    double min_output;
};


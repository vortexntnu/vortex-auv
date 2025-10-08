#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class PID_controller {
    PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output);
    double calculate_thrust(double reference, double current_position, double dt);
    void reset_controller();
    private:
    double k_p;
    double k_i;
    double k_d;
    double max_output;
    double min_output;
    double integral;
    double previous_error;
    double previous_position;
};
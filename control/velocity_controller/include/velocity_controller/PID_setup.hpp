#pragma once

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

class PID_controller {
   public:
    // PID_controller(double Kp=0, double Ki=0, double Kd=0, double
    // max_output=0, double min_output=0, double dt=0); PID_controller(double
    // k_p, double k_i, double k_d) : PID_controller(k_p, k_i, k_d, 100.0,
    // -100.0) {};
    bool calculate_thrust(double error);
    bool calculate_thrust(double error, double error_d);
    void reset_controller();
    double get_output();
    bool set_output_limits(double min_output, double max_output);
    bool set_parameters(double k_p, double k_i, double k_d, double dt);
    bool set_parameters(std::vector<double>& params, double dt);
    bool set_dt(double dt);

   private:
    double Kp_, Ki_, Kd_, dt_;
    double integral = 0;
    double previous_error = 0;
    double output = 0;
    double max_output_, min_output_;
    bool init = false;
};

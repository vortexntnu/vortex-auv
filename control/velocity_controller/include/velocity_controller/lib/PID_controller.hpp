#pragma once
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
//TODO(henrimha): Make the constructor take in a PID_params struct and make a PID_params struct in the header file
struct PID_params{
    double k_p, k_i, k_d;
    double dt;
    double max_output;
    double min_output;
    const std::vector<double>& operator=(const std::vector<double>& params);
    // Constructor from gains vector and other params
    //TODO(henrimha): change the vector to an array
    PID_params(const std::vector<double>& gains, double dt_val, double max_out, double min_out)
        : k_p(gains[0]), k_i(gains[1]), k_d(gains[2]), dt(dt_val), max_output(max_out), min_output(min_out) {}
    
    PID_params() = default;
    
};

class PID_controller {
   public:
   explicit PID_controller(PID_params params);
    /** @brief Calculates the thrust based on the error and internal parameters, with a default derivative of 0*/
   /** @brief Calculates the thrust based on the error and internal parameters*/
    double calculate_thrust(double error);
    /** @brief Calculates the thrust based on the error with external derivative */
    double calculate_thrust(double error, double error_d);
    /** @brief Resets the all internal states, inlcuding integral, output and previous error */
    void reset_controller();
    /** @brief Returns the validity of the controller */
    bool get_validity(){return valid;};
    
   private:
   /** @brief internal variables for calculation */
    PID_params params_;
    double integral = 0;
    double previous_error = 0;
    /** @brief Indicates if the controller is initialized properly*/
    bool valid = false;
};

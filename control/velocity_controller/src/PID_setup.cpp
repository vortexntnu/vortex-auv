#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/LQR_setup.hpp"
#include "velocity_controller/utilities.hpp"

PID_controller::PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output):k_p(k_p), k_i(k_i), k_d(k_d), max_output(max_output), min_output(min_output) {
    integral = 0.0;
    previous_error = 0.0;
};
void PID_controller::calculate_thrust(double error, double dt){
    //P calculation
    last_output=k_p*error;
    //D calculation
    last_output += k_d * (error - previous_error) / dt;
    //I calculation with anti-windup
    integral += error * dt;
    if (integral > max_output) {
        integral -= error * dt; //anti windup
    } else if (integral < min_output) {
        integral -= error * dt; //anti windup
    }
    last_output += k_i * integral;
    previous_error = error;
    //Output calculation with saturation

    if (last_output > max_output){
        last_output = max_output;
    }
    else if (last_output < min_output){
        last_output = min_output;
    }

    return;
}; 
void PID_controller::reset_controller(){
    integral = 0.0;
    previous_error = 0.0;
}

double PID_controller::output(){
    return last_output;
};

void PID_controller::set_output_limits(double min_output, double max_output){
    this->min_output = min_output;
    this->max_output = max_output;
    return;
};


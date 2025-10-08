#include "velocity_controller/PID_setup.hpp"

PID_controller::PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output):k_p(k_p), k_i(k_i), k_d(k_d), max_output(max_output), min_output(min_output) {
    integral = 0.0;
    previous_error = 0.0;
    previous_position = 0.0;
};
double PID_controller::calculate_thrust(double reference, double current_position, double dt){
    //Error calculation
    double error = reference - current_position;
    //P calculation
    double output=k_p*error;
    //D calculation
    output += k_d * (current_position - previous_position) / dt;
    previous_position = current_position;
    //I calculation with anti-windup
    integral += error * dt;
    if (integral > max_output) {
        integral -= error * dt; //anti windup
    } else if (integral < min_output) {
        integral -= error * dt; //anti windup
    }
    output += k_i * integral;
    previous_error = error;
    //Output calculation with saturation

    if (output > max_output){
        output = max_output;
    }
    else if (output < min_output){
        output = min_output;
    }
    return output;
};
void PID_controller::reset_controller(){
    integral = 0.0;
    previous_error = 0.0;
    previous_position = 0.0;
}
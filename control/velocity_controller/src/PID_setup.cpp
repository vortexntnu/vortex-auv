#include "velocity_controller/PID_setup.hpp"

PID_controller::PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output):k_p(k_p), k_i(k_i), k_d(k_d), max_output(max_output), min_output(min_output) {
    integral = 0.0;
    previous_error = 0.0;
    previous_position = 0.0;
};
void PID_controller::calculate_thrust(double reference, double current_position, double dt){
    //Error calculation
    double error = reference - current_position;
    //P calculation
    last_output=k_p*error;
    //D calculation
    last_output += k_d * (current_position - previous_position) / dt;
    previous_position = current_position;
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
    previous_position = 0.0;
}

double PID_controller::output(){
    return last_output;
};

void PID_controller::set_output_limits(double min_output, double max_output){
    this->min_output = min_output;
    this->max_output = max_output;
    return;
};

angle quaternion_to_euler_angle(double w, double x, double y, double z){
    double ysqr = y * y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    double phi = std::atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double theta = std::asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);
    double psi = std::atan2(t3, t4);

    return {phi, theta, psi};
};
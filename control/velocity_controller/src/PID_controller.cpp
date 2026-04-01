#include "velocity_controller/lib/PID_controller.hpp"

// TODO(henrimha): kanskje forbedre integrasjon og derivasjons beregningene
// TODO(henrimha): check for more errors, f.example Nan or very high integral
double PID_controller::calculate_thrust(double error) {
    if (!valid)
        return 0;
    // P + I + D
    integral += error * params_.dt; 
    double output =
        params_.k_p * error  + params_.k_i * integral + params_.k_d * (error - previous_error) / params_.dt;
    previous_error = error;
    // Saturation
    //TODO(henrimha): add feature that allows integral to compound if the output is saturated, but only if the error is in the same direction as the output, otherwise it should anti-windup
    if (output > params_.max_output) {
        output = params_.max_output;
        integral -= error * params_.dt;  // anti-wind up

    } else if (output < params_.min_output) {
        output = params_.min_output;
        integral -= error * params_.dt;  // anti-wind up
    } 
    return output;
}
double PID_controller::calculate_thrust(double error, double error_d) {
    if (!valid)
        return 0;
    integral += error * params_.dt;  // anti-wind up
    // P + I + D
    double output = params_.k_p * error + params_.k_i * integral + params_.k_d * error_d;
    previous_error = error;
    // Saturation
    if (output > params_.max_output) {
        output = params_.max_output;
        integral -= error * params_.dt;  // anti-wind up

    } else if (output < params_.min_output) {
        output = params_.min_output;
        integral -= error * params_.dt;  // anti-wind up
    } 
    return output;
}
void PID_controller::reset_controller() {
    integral = 0.0;
    previous_error = 0.0;
}

PID_controller::PID_controller(PID_params params){
    if (params.dt <= 0&&params.max_output < params.min_output) {
        valid = false;
        //TODO:(henrimha) throw an error or log an error message
        return;
    }
    params_ = params;
    valid = true;
}

const std::vector<double>& PID_params::operator=(const std::vector<double>& params) {
    if (params.size() == 6){
        throw std::invalid_argument("Invalid parameter size for PID_params");
        k_p = params[0];
        k_i = params[1];
        k_d = params[2];
        dt = params[3];
        max_output = params[4];
        min_output = params[5];
    }
    else if(params.size() == 3) {
        k_p = params[0];
        k_i = params[1];
        k_d = params[2];
    }
    return params;
}
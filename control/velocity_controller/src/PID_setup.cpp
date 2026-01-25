#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/LQR_setup.hpp"
#include "velocity_controller/utilities.hpp"

PID_controller::PID_controller( double k_p, double k_i, double k_d, double max_output, double min_output):k_p(k_p), k_i(k_i), k_d(k_d), max_output(max_output), min_output(min_output) {
    integral = 0.0;
    previous_error = 0.0;
};
double PID_controller::calculate_thrust(double error, double dt){
    if (dt<=0){
        return 0;
    }
    //P + I + D
    output=k_p*error+k_i*integral + k_d * (error - previous_error) / dt;

    //Saturation
    if (output>max_output){
        output = max_output;
    }
    else if (output < min_output){
        output = min_output;
    }
    else{
        integral+=error*dt; //anti-wind up
    }
    previous_error = error; 

    return output;
}; 
void PID_controller::reset_controller(){
    integral = 0.0;
    previous_error = 0.0;
    output=0.0;
}

double PID_controller::get_output(){
    return output;
};

bool PID_controller::set_output_limits(double min_output, double max_output){
    if (max_output<min_output){
        return false;
    }
    this->min_output = min_output;
    this->max_output = max_output;
    return true;
};
void PID_controller::set_parameters(double k_p,double k_i, double k_d){
    this->k_p=k_p;
    this->k_i=k_i;
    this->k_d=k_d;
}

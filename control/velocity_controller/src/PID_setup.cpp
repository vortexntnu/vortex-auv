#include "velocity_controller/PID_setup.hpp"

/*
PID_controller::PID_controller( double Kp, double Ki, double Kd, double max_output, double min_output, double dt):Kp_(Kp), Ki_(Ki), Kd_(Kd), max_output_(max_output), min_output_(min_output), dt_(dt) {
    integral = 0.0;
    previous_error = 0.0;
};*/
//TODO: kanskje forbedre integrasjon og derivasjons beregningene
//TODO: check for more errors, f.example Nan or very high intergral
bool PID_controller::calculate_thrust(double error){
    if(!init)return false;
    //Saturation
    if (output>max_output_){
        output = max_output_;
    }
    else if (output < min_output_){
        output = min_output_;
    }
    else{
        integral+=error*dt_; //anti-wind up
    }
    //P + I + D
    output=Kp_*error+Ki_*integral + Kd_ * (error - previous_error) / dt_;    
    previous_error = error; 

    return true;
}; 
bool PID_controller::calculate_thrust(double error, double error_d){
    if(!init)return false;
    //Saturation
    if (output>max_output_){
        output = max_output_;
    }
    else if (output < min_output_){
        output = min_output_;
    }
    else{
        integral+=error*dt_; //anti-wind up
    }
    //P + I + D
    output=Kp_*error+Ki_*integral + Kd_ * error_d;    
    previous_error = error; 

    return true;
}
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
    min_output_ = min_output;
    max_output_ = max_output;
    return true;
};
bool PID_controller::set_parameters(double Kp,double Ki, double Kd, double dt){
    Kp_=Kp; 
    Ki_=Ki;
    Kd_=Kd;
    if(set_dt(dt)){
        init=true;
        return true;
    };
    return false;
};

bool PID_controller::set_dt(double dt){
    if (dt<=0){
        return false;
    }
    dt_=dt;
    return true;
}
bool PID_controller::set_parameters(std::vector<double>& params,double dt){
    return set_parameters(params.at(0),params.at(1),params.at(2), dt);

};


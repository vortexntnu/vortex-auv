#include <heading_hold/HHpid.h>
#include <iostream>

HHpid::HHpid(double dt, double max, double min, double K_p, double K_d, double K_i):
 dt(dt), max(max),min(min),K_p(K_p),K_d(K_d), K_i(K_i), error(0), pre_error(0), 
 integral(0){}

HHpid::~HHpid(){}

double HHpid::calculate(){
    //P-term

    double P = K_p*error;

    integral += error*dt;

    double I = K_i*integral;

    double derivative = (error - pre_error)/dt;
    double D = K_d*derivative;
    std::cout << " P " << P << "I " << I << "D " << D << std::endl;
    double sum = P + I + D;
    if (sum > max){
        sum = max;
    }
    else if(sum < min){
        sum = min;
    }
    
     
    return sum;
}

void HHpid::updateError(double err){
    this->pre_error = this->error;
    this->error = err;
}
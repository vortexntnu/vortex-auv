#ifndef HH_PID_H
#define HH_PID_H

#include <ros/ros.h>

class HHpid{
    private:
    double dt;
    double max;
    double min;
    double K_p;
    double K_d;
    double K_i;
    double integral;
    double error;
    double pre_error;

    public:
    HHpid(double dt, double max, double min, double K_p, double K_d, double K_i);
    HHpid();
    ~HHpid();
    double calculate();
    void updateError(double err);
};

#endif
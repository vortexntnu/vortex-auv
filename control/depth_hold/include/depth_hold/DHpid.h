#ifndef DH_PID_H
#define DH_PID_H

#include <ros/ros.h>

class DHpid{
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
    DHpid(double dt, double max, double min, double K_p, double K_d, double K_i);
    DHpid();
    ~DHpid();
    double calculate();
    void updateError(double err);
};

#endif
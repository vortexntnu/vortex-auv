#ifndef CAMERA_PID_H
#define CAMERA_PID_H

#include <ros/ros.h>

class Camerapid
{
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
  Camerapid(double dt, double max, double min, double K_p, double K_d, double K_i);
  Camerapid();
  ~Camerapid();
  double calculate();
  void updateError(double err);
};

#endif

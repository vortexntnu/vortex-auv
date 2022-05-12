#ifndef DEPTH_ESTIMATOR_H
#define DEPTH_ESTIMATOR_H

#include <string>

#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>

class DepthEstimator {
public:
  DepthEstimator(ros::NodeHandle nh);

private:
  ros::NodeHandle nh;
  ros::Subscriber pressure_sub;
  ros::Publisher depth_pub;

  double atmospheric_pressure;
  double water_density;
  double earth_gravitation;

  void pressureCallback(const sensor_msgs::FluidPressure &msg);
};

#endif

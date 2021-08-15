/*
   Written by Kristoffer Rakstad Solberg & Øyvind Denvik, Students
   Copyright (c) 2019 Manta AUV, Vortex NTNU.
   All rights reserved. */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <std_msgs/Float64.h>

/* Include guard to prevent double declaration of identifiers
   such as types, enums and static variacles */

#ifndef __underwaterOdom_ROS_HH__
#define __underwaterOdom_ROS_HH__

class UnderwaterOdom
{
public:
  // Constructor
  UnderwaterOdom();

  // Destructor
  ~UnderwaterOdom(){};

  // Functions
  void depthCallback(const std_msgs::Float64& msg);
  void dvlCallback(const geometry_msgs::TwistWithCovarianceStamped& msg);

private:
  // Nodehandle

  ros::NodeHandle nh_;

  // Subscribers

  ros::Subscriber depth_sub_;
  ros::Subscriber dvl_twist_sub_;

  // Publishers

  // ros::Publisher depth_odom_pub_;
  ros::Publisher odom_pub_;

  // Messages
  nav_msgs::Odometry odom;
};

#endif  // __underwaterOdom_ROS_HH__

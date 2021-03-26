#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include "MiniPID.h"

class VelocityController
{
public:
  VelocityController(ros::NodeHandle ros_node);
  void odometryCallback(const nav_msgs::Odometry& odom_msg);
  void controlLawCallback(const geometry_msgs::Twist& twist_msg);

private:
  std::string odometry_topic;
  std::string thrust_topic;
  std::string desired_velocity_topic;
  ros::Publisher thrust_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber vel_sub;
  nav_msgs::Odometry odometry;
};

#endif

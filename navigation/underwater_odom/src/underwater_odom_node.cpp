/*
     Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "underwater_odom_node.h"

/* Constructor */
UnderwaterOdom::UnderwaterOdom()
{
  // Subscribers
  depth_sub_ = nh_.subscribe("/depth/estimated", 1, &UnderwaterOdom::depthCallback, this);
  dvl_twist_sub_ = nh_.subscribe("/dvl/twist", 1, &UnderwaterOdom::dvlCallback, this);

  // Publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/dvl/odom", 1);

  // headers
  odom.header.frame_id = "dvl/odom";
  odom.child_frame_id = "auv/base_link";
}

/* Callback */
void UnderwaterOdom::depthCallback(const std_msgs::Float64& msg)
{
  // update odom pose, ENU
  odom.pose.pose.position.z = msg.data;
}

void UnderwaterOdom::dvlCallback(const geometry_msgs::TwistWithCovarianceStamped& msg)
{
  // header timestam
  odom.header.stamp = ros::Time::now();

  // update odom twist
  odom.twist.twist.linear.x = msg.twist.twist.linear.x;
  odom.twist.twist.linear.y = msg.twist.twist.linear.y;
  odom.twist.twist.linear.z = msg.twist.twist.linear.z;

  // publish odom
  odom_pub_.publish(odom);
}

/* ROS SPIN*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "underwater_odom");
  ros::NodeHandle nh;
  UnderwaterOdom odom;
  ros::spin();
  return 0;
}

/* 
     Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "underwater_odom_node.h"

/* Constructor */
UnderwaterOdom::UnderwaterOdom(){

  // Subscribers	
  fluid_pressure_sub_ = nh_.subscribe("/auv/pressure", 1, &UnderwaterOdom::pressureCallback, this);
  dvl_twist_sub_ = nh_.subscribe("/auv/dvl_twist", 1, &UnderwaterOdom::dvlCallback, this);

  // Publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/auv/odom",1);


  // values picked from /params/environment_config.yaml
  if (!nh_.getParam("atmosphere/pressure", atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!nh_.getParam("/water/density", water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!nh_.getParam("/gravity/acceleration", earth_gravitation))
    ROS_ERROR("Could not read parameter gravititional acceleration.");
	
  // headers
	odom.header.frame_id = "auv/odom";
	odom.child_frame_id = "auv/base_link";

}


/* Callback */
void UnderwaterOdom::pressureCallback(const sensor_msgs::FluidPressure &msg){

	// compute depth
	const float gauge_pressure = 1000*msg.fluid_pressure - atmospheric_pressure;
	const float depth_meters = gauge_pressure / (water_density * earth_gravitation);
	
	// update odom pose, ENU
	odom.pose.pose.position.z = -depth_meters;

}

void UnderwaterOdom::dvlCallback(const geometry_msgs::TwistWithCovarianceStamped &msg){

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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom");
  ros::NodeHandle nh;
  UnderwaterOdom odom;
  ros::spin();
  return 0;
}

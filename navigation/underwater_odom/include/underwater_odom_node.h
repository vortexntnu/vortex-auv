#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

/* Include guard to prevent double declaration of identifiers
   such as types, enums and static variacles */

#ifndef __underwaterOdom_ROS_HH__
#define __underwaterOdom_ROS_HH__

class UnderwaterOdom {

	public:

	// Constructor
	UnderwaterOdom();

	// Functions
	void pressureCallback(const sensor_msgs::FluidPressure &msg);
	void depthEstimate(const geometry_msgs::PoseWithCovarianceStamped &msg);

	private:

	// Nodehandle	

	ros::NodeHandle nh_;	
	
	// Subscribers

	ros::Subscriber fluid_pressure_sub_;

	// Publishers

	ros::Publisher depth_odom_pub_;

	// Variables

	double atmospheric_pressure; // [kPa]
	double water_density; //[kg/m3]
	double earth_gravitation; //[m/s2]
	geometry_msgs::PoseWithCovarianceStamped depth_msg;
	std_msgs::Header base;

};


#endif // __underwaterOdom_ROS_HH__
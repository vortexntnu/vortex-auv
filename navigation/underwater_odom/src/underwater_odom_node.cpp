#include "underwater_odom_node.h"

/* Constructor */
UnderwaterOdom::UnderwaterOdom(){

  fluid_pressure_sub_ = nh_.subscribe("/manta/pressure", 1, &UnderwaterOdom::pressureCallback, this);
  depth_odom_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/manta/depth_odom", 1);


  // values picked from /params/environment_config.yaml
  if (!nh_.getParam("atmosphere/pressure", atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!nh_.getParam("/water/density", water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!nh_.getParam("/gravity/acceleration", earth_gravitation))
    ROS_ERROR("Could not read parameter gravititional acceleration.");
}


/* Callback */
void UnderwaterOdom::pressureCallback(const sensor_msgs::FluidPressure &msg){

	// compute depth
	base.frame_id = "dvl_link_NED";
	const float gauge_pressure = 1000*msg.fluid_pressure - atmospheric_pressure;
	const float depth_meters = gauge_pressure / (water_density * earth_gravitation);

	// publish
	depth_msg.header = base;
	depth_msg.pose.pose.position.z = depth_meters;
	depth_odom_pub_.publish(depth_msg);
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

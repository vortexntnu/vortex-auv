#include "depth_estimator/depth_estimator.h"

DepthEstimator::DepthEstimator(ros::NodeHandle nh) : nh(nh)
{
  // params

  if (!nh.getParam("/atmosphere/pressure", atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!nh.getParam("/water/density", water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!nh.getParam("/gravity/acceleration", earth_gravitation))
    ROS_ERROR("Could not read parameter gravititional acceleration.");

  std::string pressure_topic;
  std::string depth_topic;
  if (!nh.getParam("depth_estimator/pressure_topic", pressure_topic))
    pressure_topic = "/dvl/pressure";
  if (!nh.getParam("depth_estimator/depth_topic", depth_topic))
    pressure_topic = "/depth/estimated";

  // subscriber and publsiher
  depth_pub = nh.advertise<std_msgs::Float64>(depth_topic, 1);
  pressure_sub = nh.subscribe(pressure_topic, 1, &DepthEstimator::pressureCallback, this);
}

void DepthEstimator::pressureCallback(const sensor_msgs::FluidPressure& msg)
{
  const double gauge_pressure = 1000 * msg.fluid_pressure - atmospheric_pressure;
  const double depth_meters = gauge_pressure / (water_density * earth_gravitation);

  std_msgs::Float64 depth_msg;
  depth_msg.data = depth_meters;

  depth_pub.publish(depth_msg);
}

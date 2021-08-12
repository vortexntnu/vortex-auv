#include "depth_estimator/depth_estimator.h"

DepthEstimator::DepthEstimator(ros::NodeHandle nh) : nh(nh)
{
  // params
  std::string pressure_topic;
  std::string depth_topic;

  if (!nh.getParam("/atmosphere/pressure", atmospheric_pressure))
  {
    ROS_ERROR("Could not read parameter atmospheric pressure.");
    ros::shutdown();
  }

  if (!nh.getParam("/water/density", water_density))
  {
    ROS_ERROR("Could not read parameter water density.");
    ros::shutdown();
  }
    
  if (!nh.getParam("/gravity/acceleration", earth_gravitation))
  {
    ROS_ERROR("Could not read parameter gravititional acceleration.");
    ros::shutdown();
  }

  if (!nh.getParam("depth_estimator/pressure_topic", pressure_topic))
    pressure_topic = "/dvl/pressure";

  if (!nh.getParam("depth_estimator/depth_topic", depth_topic))
    depth_topic = "/depth/estimated";

  // subscriber and publsiher
  depth_pub = nh.advertise<std_msgs::Float32>("/depth/estimator", 1);
  pressure_sub = nh.subscribe(pressure_topic, 1, &DepthEstimator::pressureCallback, this);
}

void DepthEstimator::pressureCallback(const sensor_msgs::FluidPressure& msg)
{
  const double mbar_pressure = msg.fluid_pressure / 10.0;
  const double depth_mm = mbar_pressure * 10.197; // 10.197 is the constant to convert mbar to mmH2O
  const double depth_meters = depth_mm / 1000.0;

  std_msgs::Float32 depth_msg;
  depth_msg.data = depth_meters;
  depth_pub.publish(depth_msg);
}

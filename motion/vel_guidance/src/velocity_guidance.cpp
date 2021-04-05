#include "vel_guidance/velocity_guidance.h"

VelocityGuidance::VelocityGuidance(ros::NodeHandle nh)
{
  if (!nh.getParam("/vel_guidance/rate", rate))
    rate = 40;
  ROS_DEBUG_STREAM("Using rate: " << rate);

  if (!nh.getParam("/vel_guidance/desired_velocity_topic", velocity_topic))
    velocity_topic = "/desired_velocity";
  ROS_DEBUG_STREAM("Using velocity topic: " << velocity_topic);

  guidance_active = false;
  vel_pub = nh.advertise<geometry_msgs::Twist>(velocity_topic, 1);
  set_vel_service = 
      nh.advertiseService("/vel_guidance/set_velocity", &VelocityGuidance::setVelocity, this);
  stop_guidance_service =
      nh.advertiseService("/vel_guidance/stop_guidance", &VelocityGuidance::stopGuidance, this);

  ROS_INFO("Velocity guidance up and ready");
}

bool VelocityGuidance::setVelocity(vortex_msgs::SetVelocityRequest& req, vortex_msgs::SetVelocityResponse& res)
{
  desired_velocity = req.desired_velocity;
  guidance_active = true;
  ROS_DEBUG_STREAM("vel guidance active and set to new velocity");
  return true;
}

bool VelocityGuidance::stopGuidance(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  guidance_active = false;
  ROS_DEBUG_STREAM("vel guidance deactived");
  return true;
}

void VelocityGuidance::spin()
{
  ros::Rate ros_rate(rate);
  while (ros::ok())
  {
    if (guidance_active)
      vel_pub.publish(desired_velocity);

    ros::spinOnce();
    ros_rate.sleep();
  }
}

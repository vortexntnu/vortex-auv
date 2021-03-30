#include "vel_guidance/velocity_guidance.h"

VelocityGuidance::VelocityGuidance(ros::NodeHandle ros_node)
{
  if (!ros_node.getParam("/vel_guidance/rate", rate))
    rate = 40;

  if (!ros_node.getParam("/vel_guidance/desired_velocity_topic", velocity_topic))
    velocity_topic = "desired/velocity";

  vel_pub = ros_node.advertise<geometry_msgs::Twist>(velocity_topic, 1);
  
  set_vel_service = 
      ros_node.advertiseService("vel_guidance/set_velocity", &VelocityGuidance::setVelocity, this);
  stop_guidance_service =
      ros_node.advertiseService("vel_guidance/stop_guidance", &VelocityGuidance::stopGuidance, this);

  ROS_INFO("Velocity guidance up and ready");
}

bool VelocityGuidance::setVelocity(vortex_msgs::SetVelocityRequest& req, vortex_msgs::SetVelocityResponse& res)
{
  desired_velocity = req.desired_velocity;
  guidance_active = true;
}

bool VelocityGuidance::stopGuidance(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  guidance_active = false;
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

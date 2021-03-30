#ifndef VELOCITY_GUIDANCE
#define VELOCITY_GUIDANCE

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "vortex_msgs/SetVelocity.h"

class VelocityGuidance
{
public:
  VelocityGuidance(ros::NodeHandle ros_node);
  void spin();

private:
  bool setVelocity(vortex_msgs::SetVelocityRequest& req, vortex_msgs::SetVelocityResponse& res);
  bool stopGuidance(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  std::string velocity_topic;
  int rate;
  bool guidance_active;
  geometry_msgs::Twist desired_velocity;
  ros::Publisher vel_pub;
  ros::ServiceServer set_vel_service;
  ros::ServiceServer stop_guidance_service;
};

#endif

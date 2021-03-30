#ifndef VELOCITY_GUIDANCE
#define VELOCITY_GUIDANCE

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include "vortex_msgs/SetVelocity.h"


/**
 * @brief class that recieves desired velocities through a service and publishes them with a predefined rate 
 * as a topic until a stop-service is called. 
 * 
 */
class VelocityGuidance
{
public:
  /**
   * @brief Construct a new Velocity Guidance object
   * 
   * @param ros_node 
   */
  VelocityGuidance(ros::NodeHandle ros_node);

  /**
   * @brief contains a control loop that repeats with the objects rate until ros::ok() is false. In the loop
   * the desired velocity is publsihed depending on what service was last called. 
   * 
   */
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

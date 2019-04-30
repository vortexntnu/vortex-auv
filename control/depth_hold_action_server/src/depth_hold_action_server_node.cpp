#include <ros/ros.h>
#include "depth_hold_action_server/depth_hold_action_server_ros.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_hold_action_server");

  DepthHoldAction depthhold("depth_hold_action_server");
  ros::spin();

  return 0;
}
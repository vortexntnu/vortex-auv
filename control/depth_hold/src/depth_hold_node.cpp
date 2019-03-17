#include "depth_hold/depth_hold_ros.h"
#include <ros/ros.h>

void paramCallback(depth_hold::DepthParamsConfig &config, uint32_t level) {
  ROS_INFO("New shit");
}


int main(int argc, char **argv)
{
ros::init(argc, argv, "depth_hold");
ros::NodeHandle nh;
DepthHold depthhold(nh);
depthhold.spin();
return 0;
}

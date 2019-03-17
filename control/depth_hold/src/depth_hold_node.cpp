#include "depth_hold/depth_hold_ros.h"
#include <ros/ros.h>
//#include <dynamic_reconfigure/server.h>
//#include <depth_hold/DepthParamsConfig.h>

void paramCallback(depth_hold::DepthParamsConfig &config, uint32_t level) {
  ROS_INFO("New shit");
}


int main(int argc, char **argv)
{
ros::init(argc, argv, "depth_hold");

//dynamic_reconfigure::Server<depth_hold::DepthParamsConfig> server;
//dynamic_reconfigure::Server<depth_hold::DepthParamsConfig>::CallbackType f;

ros::NodeHandle nh;
DepthHold depthhold(nh);
//f = boost::bind(&paramCallback, _1, _2);
//server.setCallback(f);

depthhold.spin();
return 0;
}

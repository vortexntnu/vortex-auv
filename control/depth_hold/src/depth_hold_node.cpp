#include <depth_hold/depth_hold_ros.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
ros::init(argc, argv, "depth hold");
ros::NodeHandle nh;
DepthHold depthhold(nh);
depthhold.spin();
return 0;
}

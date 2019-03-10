#include "heading_hold/heading_hold_ros.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
ros::init(argc, argv, "heading_hold");
ros::NodeHandle nh;
HeadingHold headinghold(nh);
headinghold.spin();
return 0;
}

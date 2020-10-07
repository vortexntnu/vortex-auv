/* 
     Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/controller_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dp_controller");
  ros::NodeHandle nh;
  Controller controller(nh);
  controller.spin();
  return 0;
}

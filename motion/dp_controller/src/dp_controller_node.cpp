/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/dp_action_server.h"
#include "dp_controller/dp_controller_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dp_controller");
  ros::NodeHandle nh;
  Controller controller(nh, "DpAction");

  controller.spin();
  return 0;
}

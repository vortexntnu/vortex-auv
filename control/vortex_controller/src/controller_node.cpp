#include "vortex_controller/controller_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  Controller controller(nh);
  controller.spin();
  return 0;
}

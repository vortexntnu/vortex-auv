#include "velocity_controller/velocity_controller.h"

int main(int argc, char **argv) {
  const bool DEBUG_MODE = false; // debug logs are printed to console when true

  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle nh;

  if (DEBUG_MODE) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
  }

  VelocityController velocity_controller(nh);

  velocity_controller.spin();
  return 0;
}
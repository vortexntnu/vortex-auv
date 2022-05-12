#include "simple_odom.h"

int main(int argc, char **argv) {
  const bool DEBUG_MODE = true; // debug logs are printed to console when true

  ros::init(argc, argv, "simple_odometry");
  ros::NodeHandle nh;

  if (DEBUG_MODE) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
  }

  // ros::Rate rate(50);
  SimpleOdom simple_odom(nh);
  simple_odom.spin();
}
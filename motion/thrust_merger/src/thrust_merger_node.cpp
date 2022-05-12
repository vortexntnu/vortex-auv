#include "thrust_merger/thrust_merger.h"

int main(int argc, char **argv) {
  const bool DEBUG_MODE = false; // debug logs are printed to console when true

  ros::init(argc, argv, "thrust_merger");
  ros::NodeHandle nh;

  if (DEBUG_MODE) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
  }

  ThrustMerger thrust_merger(nh);
  thrust_merger.spin();
}

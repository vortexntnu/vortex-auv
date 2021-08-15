#include "vel_guidance/velocity_guidance.h"

int main(int argc, char** argv)
{
  const bool DEBUG_MODE = false;  // debug logs are printed to console when true

  ros::init(argc, argv, "velocity_guidance");
  ros::NodeHandle nh;

  if (DEBUG_MODE)
  {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
  }

  VelocityGuidance velocity_guidance(nh);
  velocity_guidance.spin();
}

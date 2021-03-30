#include "vel_guidance/velocity_guidance.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_guidance");
  ros::NodeHandle ros_node;

  VelocityGuidance velocity_guidance(ros_node);
  velocity_guidance.spin();
}

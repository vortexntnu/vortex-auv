#include "vortex_estimator/simple_estimator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator");
  ros::NodeHandle nh;
  SimpleEstimator estimator;
  ros::spin();
  return 0;
}

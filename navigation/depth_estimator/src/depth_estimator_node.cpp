#include "depth_estimator/depth_estimator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_estimator");
  ros::NodeHandle nh;
  DepthEstimator depth_estimator(nh);
  ros::spin();
}

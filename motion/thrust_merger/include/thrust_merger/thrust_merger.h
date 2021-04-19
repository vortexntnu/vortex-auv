#ifndef THRUST_MERGER_H
#define THRUST_MERGER_H

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Wrench.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// These typdefs are lacking from the default eigen namespace
namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

class ThrustMerger
{
public:
  ThrustMerger(ros::NodeHandle nh);
  void spin();

private:
  ros::NodeHandle nh;
  double rate;

  ros::Subscriber dp_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber los_sub;
  ros::Subscriber joy_sub;
  ros::Publisher thrust_pub;

  Eigen::Vector6d dp_wrench;
  Eigen::Vector6d los_wrench;
  Eigen::Vector6d vel_wrench;
  Eigen::Vector6d joy_wrench;
  Eigen::Vector6d combined_wrench;

  int spins_without_update_limit;
  int dp_counter;
  int los_counter;
  int vel_counter;
  int joy_counter;

  void dpCallback(geometry_msgs::Wrench wrench_msg);
  void losCallback(geometry_msgs::Wrench wrench_msg);
  void velCallback(geometry_msgs::Wrench wrench_msg);
  void joyCallback(geometry_msgs::Wrench wrench_msg);
};

#endif

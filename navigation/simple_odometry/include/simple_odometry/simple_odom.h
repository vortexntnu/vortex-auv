#ifndef SIMPLE_ODOM_H
#define SIMPLE_ODOM_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

/**
 * @brief Class that combines measurements from IMU and DVL into a simple
 * odometry publisher. Positions in x and y are estimated by euler integration.
 *
 */
class SimpleOdom
{
public:
  SimpleOdom(ros::NodeHandle nh);
  void spin();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  void imuCallback(const sensor_msgs::Imu& imu_msg);
  void dvlCallback(const nav_msgs::Odometry& odom_msg);
  Eigen::Vector3d linear_vel;
  Eigen::Vector3d angular_vel;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  ros::Subscriber imu_sub;
  ros::Subscriber dvl_sub;
  ros::Publisher odom_pub;
  ros::NodeHandle nh;
  double update_rate;
};

#endif

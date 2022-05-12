#ifndef SIMPLE_ODOM_H
#define SIMPLE_ODOM_H

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief Class that combines measurements from IMU and DVL into a simple
 * odometry publisher. Positions in x and y are estimated by euler integration.
 *
 */
class SimpleOdom {
public:
  SimpleOdom(ros::NodeHandle nh);
  void spin();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  void imuCallback(const sensor_msgs::Imu &imu_msg);
  void dvlCallback(const geometry_msgs::TwistWithCovarianceStamped &twist_msg);
  void mocapCallback(const geometry_msgs::PoseStamped &msg);
  Eigen::Vector3d position;
  tf2::Quaternion orientation;
  tf2::Vector3 linear_vel;
  tf2::Vector3 angular_vel;
  tf2::Quaternion imu_rotation;
  tf2::Quaternion dvl_rotation;
  tf2::Vector3 dvl_translation;
  ros::Subscriber imu_sub;
  ros::Subscriber dvl_sub;
  ros::Subscriber mocap_sub;
  ros::Publisher odom_pub;
  ros::NodeHandle nh;
  double update_rate;
};

#endif

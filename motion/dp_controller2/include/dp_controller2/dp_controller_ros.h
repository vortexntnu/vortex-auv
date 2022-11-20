
/*   Written by Kristoffer Rakstad Solberg, Student
     Documented by Christopher Strøm
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief A ROS wrapper layer for the quaternion PID controller
 *
 */
#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <map>
#include <math.h>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <actionlib/server/simple_action_server.h>
//#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
//#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include "eigen_typedefs.h"

#include "dp_controller2/quaternion_dp_controller.h"





/**
 * @brief the Controller class
 *
 * This class serves as a wrapper for the lower-level controller implementation
 * @see quaternion_pd_controller.h
 *
 */
class Controller {
public:

  /**
   * @brief Controller class constructor
   *
   * @param nh ROS nodehandle
   */
  explicit Controller(ros::NodeHandle nh);

  /**
   * @brief Callback for the odometry subscriber
   *
   * @param msg   A nav_msg::Odometry message containing state data about the
   * AUV.
   */
  void odometryCallback(const nav_msgs::Odometry &msg);
  Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw);
  Eigen::Vector3d QuaterniondToEuler(Eigen::Quaterniond q);
  void spin();

private:
  ros::NodeHandle m_nh; /** Nodehandle          */

  ros::Subscriber m_odometry_sub;    /** Odometry subscriber    */

  ros::Publisher m_wrench_pub; /** Wrench publisher    */

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d position;       /** Current position      */
  Eigen::Quaterniond orientation; /** Current orientation   */
  Eigen::Vector6d velocity;       /** Current velocity      */


  QuaternionPIDController m_controller;
  

};

#endif // VORTEX_CONTROLLER_CONTROLLER_ROS_H

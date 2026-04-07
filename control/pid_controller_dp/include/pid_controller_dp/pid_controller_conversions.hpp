#ifndef PID_CONTROLLER_DP__PID_CONTROLLER_CONVERSIONS_HPP_
#define PID_CONTROLLER_DP__PID_CONTROLLER_CONVERSIONS_HPP_

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "pid_controller_dp/typedefs.hpp"

/**
 * @brief Convert a ROS PoseWithCovariance message to an Eta pose struct.
 * @param msg ROS PoseWithCovariance message
 * @return Eta struct containing position and quaternion orientation
 */
types::Eta eta_convert_from_ros_to_eigen(
    const geometry_msgs::msg::PoseWithCovariance& msg);

/**
 * @brief Convert a ROS TwistWithCovariance message to a Nu velocity struct.
 * @param msg ROS TwistWithCovariance message
 * @return Nu struct containing linear and angular velocity
 */
types::Nu nu_convert_from_ros_to_eigen(
    const geometry_msgs::msg::TwistWithCovariance& msg);

#endif  // PID_CONTROLLER_DP__PID_CONTROLLER_CONVERSIONS_HPP_

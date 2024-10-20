#ifndef PID_UTILS_HPP
#define PID_UTILS_HPP

#include <std_msgs/msg/float64_multi_array.hpp>
#include "pid_controller_dp/eigen_typedefs.hpp"
#include <eigen3/Eigen/Geometry>
#include <cmath>

Eigen::Matrix6d float64multiarray_to_diagonal_matrix6d(const std_msgs::msg::Float64MultiArray &msg);

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &quat);

Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d &euler_angles);

double ssa(double angle);

Eigen::Vector6d apply_ssa(const Eigen::Vector6d &eta);

Eigen::Matrix3d calculate_R(const Eigen::Vector6d &eta);

Eigen::Matrix3d calculate_T(const Eigen::Vector6d &eta);

Eigen::Matrix6d calculate_J(const Eigen::Vector6d &eta);

#endif
#ifndef PID_UTILS_HPP
#define PID_UTILS_HPP

#include "pid_controller_dp/eigen_typedefs.hpp"
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

Eigen::Matrix6d float64multiarray_to_diagonal_matrix6d(const std_msgs::msg::Float64MultiArray &msg);

double ssa(double angle);

// Eigen::Vector6d apply_ssa(const Eigen::Vector7d &eta);

Eigen::Matrix3d calculate_R_quat(const Eigen::Vector4d &q);

// Eigen::Matrix3d calculate_T(const Eigen::Vector7d &eta);

Eigen::Matrix4x3d calculate_T_quat(const Eigen::Vector4d &q);

Eigen::Matrix6x7d calculate_J_sudo_inv(const Eigen::Vector7d &eta);

Eigen::Vector7d error_eta(const Eigen::Vector7d &eta, const Eigen::Vector7d &eta_d);

Eigen::Matrix7x6d calculate_J(const Eigen::Vector7d &eta);

Eigen::Vector7d anti_windup(const double dt, const Eigen::Vector7d &error, const Eigen::Vector7d &integral);

Eigen::Vector6d limit_input(const Eigen::Vector6d &input);

#endif

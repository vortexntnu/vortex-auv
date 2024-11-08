#ifndef PID_UTILS_HPP
#define PID_UTILS_HPP

#include "pid_controller_dp/eigen_typedefs.hpp"
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// @brief Calculate the sine of an angle in degrees
// @param angle: Angle in degrees
// @return Smallest sine angle of the input
double ssa(double angle);

// @brief Convert a Float64MultiArray message to a 6x6 diagonal matrix
// @param msg: Float64MultiArray message containing 6 elements
// @return 6x6 diagonal matrix with the elements of the message on the diagonal
Eigen::Matrix6d float64multiarray_to_diagonal_matrix6d(const std_msgs::msg::Float64MultiArray &msg);

// @brief Calculate the rotation matrix from a quaternion
// @param q: Quaternion represented as a 4D vector [w, x, y, z]
// @return 3x3 rotation matrix
Eigen::Matrix3d calculate_R_quat(const Eigen::Vector4d &q);

// @brief Calculate the transformation matrix from a quaternion
// @param q: Quaternion represented as a 4D vector [w, x, y, z]
// @return 4x3 transformation matrix
Eigen::Matrix4x3d calculate_T_quat(const Eigen::Vector4d &q);

// @brief Calculate the Jacobian matrix
// @param eta: 7D vector containing the vehicle pose [x, y, z, w, x, y, z]
// @return 7x6 Jacobian matrix
Eigen::Matrix7x6d calculate_J(const Eigen::Vector7d &eta);

// @brief Calculate the pseudo-inverse of the Jacobian matrix
// @param eta: 7D vector containing the vehicle pose [x, y, z, w, x, y, z]
// @return 6x7 pseudo-inverse Jacobian matrix
Eigen::Matrix6x7d calculate_J_sudo_inv(const Eigen::Vector7d &eta);

// @brief Calculate the error between the desired and actual vehicle pose
// @param eta: 7D vector containing the actual vehicle pose [x, y, z, w, x, y, z]
// @param eta_d: 7D vector containing the desired vehicle pose [x, y, z, w, x, y, z]
// @return 7D vector containing the error between the desired and actual vehicle pose
Eigen::Vector7d error_eta(const Eigen::Vector7d &eta, const Eigen::Vector7d &eta_d);

// @brief Calculate the anti-windup term
// @param dt: Time step
// @param error: 7D vector containing the error between the desired and actual vehicle pose [x, y, z, w, x, y, z]
// @param integral: 7D vector containing the integral term of the PID controller [x, y, z, w, x, y, z]
// @return 7D vector containing the anti-windup term
Eigen::Vector7d anti_windup(const double dt, const Eigen::Vector7d &error, const Eigen::Vector7d &integral);

// @brief Limit the input to the PID controller
// @param input: 6D vector containing the input to the PID controller [u, v, w, p, q, r]
// @return 6D vector containing the limited input to the PID controller [u, v, w, p, q, r]
Eigen::Vector6d limit_input(const Eigen::Vector6d &input);

#endif

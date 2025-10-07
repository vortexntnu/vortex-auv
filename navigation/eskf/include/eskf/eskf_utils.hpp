#ifndef ESKF_UTILS_HPP
#define ESKF_UTILS_HPP

#include <cmath>
#include "eigen3/Eigen/Dense"
#include "eskf/typedefs.hpp"

// @brief Compute the skew-symmetric matrix of a vector
// @param v: Input vector
// @return Skew-symmetric matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

// @brief Square a value
// @param value: Input value
// @return Squared value
double sq(const double& value);

// @brief Normalize an angle to the range [-pi, pi]
// @param angle: Input angle in radians
// @return Normalized angle in radians
double ssa(const double& angle);

// @brief Calculate the transformation matrix using a quaternion
// @param quat: Input quaternion
// @return Transformation matrix
Eigen::Matrix4x3d calculate_T_q(const Eigen::Quaterniond& quat);

// @brief Convert a rotation vector to a quaternion
// @param vector: Input rotation vector
// @return Corresponding quaternion
Eigen::Quaterniond vector3d_to_quaternion(const Eigen::Vector3d& vector);

// @brief Convert Euler angles to a quaternion
// @param euler: Input Euler angles (roll, pitch, yaw) in radians
// @return Corresponding quaternion
Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& euler);

#endif  // ESKF_UTILS_HPP

#ifndef ESKF_UTILS_HPP
#define ESKF_UTILS_HPP

#include <cmath>
#include "eigen3/Eigen/Dense"
#include "eskf/typedefs.hpp"

Eigen::Matrix3d skew(const Eigen::Vector3d& v);

double sq(const double& value);

double ssa(const double& angle);

Eigen::Quaterniond vector3d_to_quaternion(const Eigen::Vector3d& vector);

Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& euler);

#endif  // ESKF_UTILS_HPP

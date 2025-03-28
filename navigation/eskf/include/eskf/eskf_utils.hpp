#ifndef ESKF_UTILS_HPP
#define ESKF_UTILS_HPP

#include "eigen3/Eigen/Dense"
#include "eskf/typedefs.hpp"

Eigen::Matrix3d skew(const Eigen::Vector3d& v);

double sq(const double& value);

#endif  // ESKF_UTILS_HPP

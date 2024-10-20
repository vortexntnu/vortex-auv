#ifndef REFERENCE_FILTER_UTILS_HPP
#define REFERENCE_FILTER_UTILS_HPP

#include <reference_filter_dp/eigen_typedefs.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &quat);

Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d &euler_angles);

#endif
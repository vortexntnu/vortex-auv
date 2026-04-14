/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector and a 6x6 Eigen matrix.
 */

#ifndef PID_CONTROLLER_DP_EULER__TYPEDEFS_HPP_
#define PID_CONTROLLER_DP_EULER__TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

using Eta = ::vortex::utils::types::PoseEuler;
using Nu = ::vortex::utils::types::Twist;

#endif  // PID_CONTROLLER_DP_EULER__TYPEDEFS_HPP_

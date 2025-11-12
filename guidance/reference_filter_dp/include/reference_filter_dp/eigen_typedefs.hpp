/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector, 6x6 Eigen matrix.
 * and 18x18 Eigen matrix.
 */

#ifndef REFERENCE_FILTER_DP__EIGEN_TYPEDEFS_HPP_
#define REFERENCE_FILTER_DP__EIGEN_TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 18, 6> Matrix18x6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 18, 1> Vector18d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

#endif  // REFERENCE_FILTER_DP__EIGEN_TYPEDEFS_HPP_

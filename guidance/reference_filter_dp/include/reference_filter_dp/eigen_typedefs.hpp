/**
 * @file eigen_typedefs.hpp
 * @brief Contains Eigen typedefs used in this package.
 */

#ifndef REFERENCE_FILTER_DP__EIGEN_TYPEDEFS_HPP_
#define REFERENCE_FILTER_DP__EIGEN_TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>

namespace Eigen {

typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 18, 6> Matrix18x6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 18, 1> Vector18d;

}  // namespace Eigen

#endif  // REFERENCE_FILTER_DP__EIGEN_TYPEDEFS_HPP_

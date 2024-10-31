/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector and a 6x6 Eigen matrix.
 */

#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <eigen3/Eigen/Dense>

namespace Eigen {
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 3, 3> Matrix3d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 7, 1> Vector7d;
  typedef Eigen::Matrix<double, 4, 1> Vector4d;
  typedef Eigen::Matrix<double, 4, 3> Matrix4x3d;
  typedef Eigen::Matrix<double, 7, 6> Matrix7x6d;
  typedef Eigen::Matrix<double, 6, 7> Matrix6x7d;
  typedef Eigen::Matrix<double, 7,7> Matrix7d;
} // namespace Eigen

#endif

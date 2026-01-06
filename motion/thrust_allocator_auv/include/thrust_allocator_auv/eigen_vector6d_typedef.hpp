/**
 * @file eigen_vector6d_typedef.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector.
 */

#ifndef THRUST_ALLOCATOR_AUV__EIGEN_VECTOR6D_TYPEDEF_HPP_
#define THRUST_ALLOCATOR_AUV__EIGEN_VECTOR6D_TYPEDEF_HPP_

#include <eigen3/Eigen/Dense>

namespace Eigen {
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

#endif  // THRUST_ALLOCATOR_AUV__EIGEN_VECTOR6D_TYPEDEF_HPP_

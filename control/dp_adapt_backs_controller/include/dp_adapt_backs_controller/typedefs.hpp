/**
 * @file typedefs.hpp
 * @brief Contains the Eigen typedefs for the controller.
 */

#ifndef DP_ADAPT_BACKS_CONTROLLER__TYPEDEFS_HPP_
#define DP_ADAPT_BACKS_CONTROLLER__TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>

namespace Eigen {

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 12> Matrix6x12d;
typedef Eigen::Matrix<double, 12, 6> Matrix12x6d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;

}  // namespace Eigen

#endif  // DP_ADAPT_BACKS_CONTROLLER__TYPEDEFS_HPP_

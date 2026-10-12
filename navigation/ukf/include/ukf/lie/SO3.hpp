/**
 * @file SO3.hpp
 * @brief Contains the exponential and logarithmic maps of SO(3).
 */

#ifndef UKF__LIE__SO3_HPP_
#define UKF__LIE__SO3_HPP_

#include <Eigen/Geometry>

/**
 * @brief Operations on the rotation group SO(3), represented by unit
 * quaternions.
 */
namespace SO3 {

/**
 * @brief The capital Exponential map, taking a rotation vector to a rotation.
 * @param rotation_vector The rotation vector (axis times angle).
 * @return The unit quaternion of the rotation.
 */
Eigen::Quaterniond Exp(const Eigen::Vector3d& rotation_vector);

/**
 * @brief The capital Logarithmic map, taking a rotation to a rotation vector.
 * @param quaternion The unit quaternion of the rotation.
 * @return The rotation vector (axis times angle).
 */
Eigen::Vector3d Log(const Eigen::Quaterniond& quaternion);

}  // namespace SO3

#endif  // UKF__LIE__SO3_HPP_

#ifndef REFERENCE_FILTER_UTILS_HPP
#define REFERENCE_FILTER_UTILS_HPP

#include <reference_filter_dp/eigen_typedefs.hpp>

// @brief Calculate the rotation matrix from the Euler angles
// @param eta The Euler angles
// @return The rotation matrix
Matrix3d calculate_R(const Vector6d &eta);

// @brief Calculate the transformation matrix from the Euler angles
// @param eta The Euler angles
// @return The transformation matrix
Matrix3d calculate_T(const Vector6d &eta);

// @brief Calculate the Jacobian matrix from the Euler angles
// @param eta The Euler angles
// @return The Jacobian matrix
Matrix6d calculate_J(const Vector6d &eta);

// @brief Calculate the shortest signed angle
// @param angle The angle
// @return The shortest signed angle
double ssa(double angle);

#endif

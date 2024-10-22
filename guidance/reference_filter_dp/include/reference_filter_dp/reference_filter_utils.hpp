#ifndef REFERENCE_FILTER_UTILS_HPP
#define REFERENCE_FILTER_UTILS_HPP

#include <reference_filter_dp/eigen_typedefs.hpp>

Matrix3d calculate_R(const Vector6d &eta);

Matrix3d calculate_T(const Vector6d &eta);

Matrix6d calculate_J(const Vector6d &eta);

double ssa(double angle);

#endif
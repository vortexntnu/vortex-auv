#ifndef GUIDANCE_VTF_UTILS_HPP
#define GUIDANCE_VTF_UTILS_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <guidance_vtf/typedefs.hpp>

Matrix3d R_z(double rotation);

double ssa(double angle);

#endif  // GUIDANCE_VTF_UTILS_HPP

/*   Written by Jae Hyeong Hwang
     Copyright (c) 2021 Manta AUV, Vortex NTNU.
     All rights reserved. */
#ifndef VORTEX_CONTROLLER_REFERENCE_MODEL_H
#define VORTEX_CONTROLLER_REFERENCE_MODEL_H
#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;


double ReferenceModel(const Egien::Vector3d &x, const Eigen::Vector3d &x_ref);

#endif  // VORTEX_CONTROLLER_REFERENCE_MODEL_H
/*   Written by Jae Hyeong Hwang
     Copyright (c) 2021 Manta AUV, Vortex NTNU.
     All rights reserved. */
#ifndef VORTEX_CONTROLLER_REFERENCE_MODEL_H
#define VORTEX_CONTROLLER_REFERENCE_MODEL_H
#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;

struct ReferenceModel {   

     ReferenceModel();

     Eigen::Vector3d calculate_smooth(const Eigen::Vector3d &x_ref);
     void reset(Eigen::Vector3d pos);

     Eigen::Vector3d x_d_prev;         /** Previous desired body position            */
     Eigen::Vector3d x_d_prev_prev;    /** Previous previous desired body position   */
     Eigen::Vector3d x_ref_prev;       /** Previous reference body position          */
     Eigen::Vector3d x_ref_prev_prev;  /** Previous previous reference body position */

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // VORTEX_CONTROLLER_REFERENCE_MODEL_H
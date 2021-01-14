/*   Written by Jae Hyeong Hwang
     Copyright (c) 2021 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/quaternion_pd_controller.h"
#include "dp_controller/reference_model.h"


ReferenceModel::ReferenceModel() {
     Eigen::Vector3d x_d_prev          = Eigen::Vector3d::Zero();
     Eigen::Vector3d x_d_prev_prev     = Eigen::Vector3d::Zero();
     Eigen::Vector3d x_ref_prev        = Eigen::Vector3d::Zero();
     Eigen::Vector3d x_ref_prev_prev   = Eigen::Vector3d::Zero();
}

Eigen::Vector3d ReferenceModel::calculate_smooth(const Eigen::Vector3d &x_ref)
{
     Eigen::Vector3d x_d;
     Eigen::Vector3d a_x(1,-1.990024937655860,0.990049813123053);
     Eigen::Vector3d b_x(6.218866798092052e-06,1.243773359618410e-05,6.218866798092052e-06);
     x_d = b_x(0) * x_ref + b_x(1) * x_ref_prev + b_x(2) * x_ref_prev_prev - a_x(1) * x_d_prev - a_x(2) * x_d_prev_prev;
     
     x_ref_prev_prev = x_ref_prev;
     x_ref_prev = x_ref;
     x_d_prev_prev = x_d_prev;
     x_d_prev = x_d;
     return x_d;
}

void ReferenceModel::reset(Eigen::Vector3d pos) {
     x_d_prev = pos;
     x_d_prev_prev = pos;
     x_ref_prev = pos;
     x_ref_prev_prev = pos;
}
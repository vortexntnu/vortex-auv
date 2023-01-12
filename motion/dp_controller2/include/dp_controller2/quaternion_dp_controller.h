/*   Written by Kristoffer Rakstad Solberg, Student
     Documentation written by Jae Hyeong Hwang and
     copied to Doxygen format by Christopher Strøm
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief A nonlinear PID controller for position and orientation
 *
 * An implementation of Fjellstad & Fossen 1994: Quaternion
 * Feedback Regulation of Underwater Vehicles, a nonlinear PD
 * position and orientation controller. All variables are named
 * as they appear in the paper.
 *
 * Note that the PD controller as described in the paper has been
 * expanded to a PID controller.
 */

#ifndef VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H
#define VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <math.h>

#include "eigen_typedefs.h"

class QuaternionPIDController{
public:

  explicit QuaternionPIDController();
  /**
   * @brief a getter for the feedback vector
   *
   * @param x     The current body position, as a 3d vector
   * @param q     The current attitude, in quaternion form
   *
   * @param nu    Body fixed linear velocity and angular velocity in x,y,z
   *
   * @param x_d   The desired body position, as a 3d vector
   * @param q_d   The desired attitude, in quaternion form
   *
   * @return Control vector without restoring forces
   */
  Eigen::Vector6d getFeedback(const Eigen::Vector3d &x,
                              const Eigen::Quaterniond &q,
                              const Eigen::Vector6d &nu,
                              const Eigen::Matrix7d eta_dot_d,
                              const Eigen::Vector3d &x_d,
                              const Eigen::Quaterniond &q_d);

  int sgn(double x);
  Eigen::Vector6d errorVector(const Eigen::Vector3d &x,
                                                       const Eigen::Vector3d &x_d,
                                                       const Eigen::Quaterniond &q, 
                                                       const Eigen::Quaterniond &q_d);
  
  Eigen::Matrix6d proportionalGainMatrix(const Eigen::Matrix3d R);
};


#endif // VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H





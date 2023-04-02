/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
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

class QuaternionPIDController {
private:
  float m_W;
  float m_B;
  Eigen::Vector3d m_r_B;
  Eigen::Vector3d m_r_G;
  Eigen::Vector6d m_p_gain;
  Eigen::Matrix6d m_i_gain;
  Eigen::Vector6d m_d_gain;
  Eigen::Vector6d m_integral;
  Eigen::Vector6d m_scale_g;

  Eigen::Vector6d m_integralAntiWindup;

public:
  explicit QuaternionPIDController(); // float W, float B, Eigen::Vector3d r_G,
                                      // Eigen::Vector3d r_B);
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
                              const Eigen::Vector6d &nu_d,
                              const Eigen::Vector3d &eta_d_pos,
                              const Eigen::Quaterniond &eta_d_ori);

  int sgn(double x);
  Eigen::Vector6d errorVector(const Eigen::Vector3d &x,
                              const Eigen::Vector3d &x_d,
                              const Eigen::Quaterniond &q,
                              const Eigen::Quaterniond &q_d);

  Eigen::Matrix6d proportionalGainMatrix(const Eigen::Matrix3d R);

  Eigen::Vector6d restoringForceVector(const Eigen::Matrix3d R);

  void init(const double W, const double B, const Eigen::Vector3d &r_G,
            const Eigen::Vector3d &r_B);
  void update_gain(Eigen::Vector6d p_gain, Eigen::Vector6d i_gain,
                   Eigen::Vector6d d_gain);

  Eigen::Vector6d P_debug = Eigen::Vector6d::Zero();
  Eigen::Vector6d I_debug = Eigen::Vector6d::Zero();
  Eigen::Vector6d D_debug = Eigen::Vector6d::Zero();
  Eigen::Vector6d g_debug = Eigen::Vector6d::Zero();
};

#endif // VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H

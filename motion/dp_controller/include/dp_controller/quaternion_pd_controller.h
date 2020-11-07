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
*/

#ifndef VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H
#define VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H
#include <math.h>
#include <cstdlib>
#include <Eigen/Dense>
#include <cmath>
#include "eigen_typedefs.h"
#include <iostream>
using namespace Eigen;

class QuaternionPdController
{
public:
  QuaternionPdController(double a, double b, double c, double i, double W, double B,
                         const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B);

  void setGains(double a, double b, double c, double i);

  // Return restoring forces vector
  Eigen::Vector6d getRestoring(const Eigen::Quaterniond &q);

  // Return control vector without restoring forces
  Eigen::Vector6d getFeedback(const Eigen::Vector3d    &x,
                              const Eigen::Quaterniond &q,
                              const Eigen::Vector6d    &nu,
                              const Eigen::Vector3d    &x_d,
                              const Eigen::Quaterniond &q_d);

  Eigen::Vector3d referenceModel(const Eigen::Vector3d   &x,
                                 const Eigen::Vector3d   &x_ref);

  bool circleOfAcceptance(const Eigen::Vector3d   &x,
                          const Eigen::Vector3d   &x_d,
                                float             R);

  // Integral error vector
  Eigen::Vector6d integral          = Eigen::Vector6d::Zero(); 
  Eigen::Vector3d x_d_prev          = Eigen::Vector3d::Zero();
  Eigen::Vector3d x_d_prev_prev     = Eigen::Vector3d::Zero();
  Eigen::Vector3d x_ref_prev        = Eigen::Vector3d::Zero();  
  Eigen::Vector3d x_ref_prev_prev   = Eigen::Vector3d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Matrix6d proportionalGainMatrix(const Eigen::Matrix3d R);
  Eigen::Matrix6d integralGainMatrix(const Eigen::Matrix3d R);
  void integralWindUp(Eigen::Vector6d &vec, double pose_lim, double att_lim);

  /**
   * @brief Calculate error vector.
   * 
   * The quaternion error is given by the product of desired state q_d and
   * current state q
   * 
   * @param x
   * @param x_d
   * 
   * @param q
   * @param q_d
   * 
   * @return a 6x1 vector consisting of the error in position and attitudes. 
  */
  Eigen::Vector6d errorVector(const Eigen::Vector3d    &x,
                              const Eigen::Vector3d    &x_d,
                              const Eigen::Quaterniond &q,
                              const Eigen::Quaterniond &q_d);

  /**
   * @brief calculate the restoring force vector
   * 
   * @param R   Rotation matrix from inertial to body frame
   * 
   * @return The restoring force vector
  */
  Eigen::Vector6d restoringForceVector(const Eigen::Matrix3d R);

  /**
   * @brief The mathematical sign-operator
   * 
   * @param x   The value whos sign is to be determined
   * 
   * @return -1 if @p x is negative, and 1 if @p x is zero or greater
   * 
  */
  int sgn(double x);

  double m_c;             /** Orientation gain            */
  double m_c_i;           /** Orientation integral gain   */
  Eigen::Matrix6d m_K_d;  /** Derivative gain matrix      */
  Eigen::Matrix3d m_K_x;  /** Position error gain matrix  */
  Eigen::Matrix3d m_K_i;  /** Integral gain matrix        */

  Eigen::Vector3d m_r_G;  /** Center of gravity, expressed in body frame  */
  Eigen::Vector3d m_r_B;  /** Center of buoyancy, expressed in body frame */
  double m_W;             /** [N] Weight of drone                         */
  double m_B;             /** [N] Buoyancy of drone                       */

};


#endif  // VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H

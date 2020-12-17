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
#include <math.h>
#include <cstdlib>
#include <Eigen/Dense>
#include <cmath>
#include "eigen_typedefs.h"
#include <iostream>
using namespace Eigen;

/**
 * @brief A complete class for the nonlinear PID controller
*/
class QuaternionPdController
{
public:

  /**
   * @brief Controller class constructor
   * 
   * @param a     Diagonal value for derivative gain matrix
   * @param b     Diagonal value for position gain matrix
   * @param c     Orientation gain
   * @param i     Scalar for orientation integral gain and diagonal value for integral gain matrix
   * @param W     Weight of drone
   * @param B     Buoyancy of drone
   * @param r_G   Center of gravity, expressed in body frame
   * @param r_B   Center of buoyancy, expressed in body frame
   * 
  */
  QuaternionPdController(double a, double b, double c, double i, double W, double B,
                         const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B);


  /**
   * @brief set the gains of the controller
   * 
   * @param a     Diagonal value for derivative gain matrix
   * @param b     Diagonal value for position gain matrix
   * @param c     Orientation gain
   * @param i     Scalar for orientation integral gain and diagonal value for integral gain matrix
   * 
  */
  void setGains(double a, double b, double c, double i);

  /**
   * @brief a getter for the restoring forces vector.
   * @see  restoringForceVector()
   *
   * @param q  A quaternion containing the AUV pose in the inertial/world frame
   * 
   * @return the restoring forces vector
  */
  Eigen::Vector6d getRestoring(const Eigen::Quaterniond &q);


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
  Eigen::Vector6d getFeedback(const Eigen::Vector3d    &x,
                              const Eigen::Quaterniond &q,
                              const Eigen::Vector6d    &nu,
                              const Eigen::Vector3d    &x_d,
                              const Eigen::Quaterniond &q_d);


  /**
   * @brief Utilize a calculated reference model to find next desired body position
   * 
   * @param x       The current body position, as a 3d vector
   * @param x_ref   The body position reference, as a 3d vector
   * 
   * @return Next desired body position
  */
  Eigen::Vector3d referenceModel(const Eigen::Vector3d   &x,
                                 const Eigen::Vector3d   &x_ref);


  /**
   * @brief Check if desired position is within a defined radius of the current position
   * 
   * @param x     The current body position, as a 3d vector
   * @param x_d   The desired body position, as a 3d vector
   * @param R     The radius of the circle of acceptance
   * 
   * @return true if inside circle of acceptance, false if not.
  */
  bool circleOfAcceptance(const Eigen::Vector3d   &x,
                          const Eigen::Vector3d   &x_d,
                                float             R);


  Eigen::Vector6d integral          = Eigen::Vector6d::Zero();  /** Integral error vector                     */
  Eigen::Vector3d x_d_prev          = Eigen::Vector3d::Zero();  /** Previous desired body position            */
  Eigen::Vector3d x_d_prev_prev     = Eigen::Vector3d::Zero();  /** Previous previous desired body position   */
  Eigen::Vector3d x_ref_prev        = Eigen::Vector3d::Zero();  /** Previous reference body position          */
  Eigen::Vector3d x_ref_prev_prev   = Eigen::Vector3d::Zero();  /** Previous previous reference body position */

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /**
   * @brief Calculate the proportional gain matrix
   * 
   * @param R   Rotation matrix from inertial to body frame
   * 
   * @return The calculated proportional gain matrix
  */
  Eigen::Matrix6d proportionalGainMatrix(const Eigen::Matrix3d R);


  /**
   * @brief Calculate the integral gain matrix
   * 
   * @param R   Rotation matrix from inertial to body frame
   * 
   * @return The calculated integral gain matrix
  */
  Eigen::Matrix6d integralGainMatrix(const Eigen::Matrix3d R);


  /**
   * @brief Prevent integral windup
   * 
   * Integrator anti-windup is needed in order to avoid that the integrator
   * integrates up beyond the saturation limits of the actuators. 
   * 
   * @see getFeedback() where @p pose_lim and @p att_lim are defined
   * 
   * @param vec         The current integral gain
   * @param pose_lim    The maximum allowed pose gain
   * @param att_lim     The maximum allowed attitude gain
   * 
  */
  void integralWindUp(Eigen::Vector6d &vec, double pose_lim, double att_lim);


  /**
   * @brief Calculate error vector.
   * 
   * The quaternion error is given by the product of desired state and
   * current state. The position error meanwhile, is given by the difference
   * between the desired position and the current position
   * 
   * @param x     The current body position, as a 3d vector
   * @param x_d   The desired body position, as a 3d vector
   * 
   * @param q     The current attitude, in quaternion form
   * @param q_d   The desired attitude, in quaternion form
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

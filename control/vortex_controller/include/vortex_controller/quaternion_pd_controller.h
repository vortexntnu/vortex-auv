// An implementation of
// Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles,
// a nonlinear PD position and orientation controller.
// All variables are named as they appear in the paper.

#ifndef VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H
#define VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H

#include <Eigen/Dense>
#include "vortex/eigen_typedefs.h"

class QuaternionPdController
{
public:
  QuaternionPdController(double a, double b, double c, double W, double B,
                         const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B);
  void setGains(double a, double b, double c);

  // Return restoring forces vector
  Eigen::Vector6d getRestoring(const Eigen::Quaterniond &q);

  // Return control vector without restoring forces
  Eigen::Vector6d getFeedback(const Eigen::Vector3d    &x,
                              const Eigen::Quaterniond &q,
                              const Eigen::Vector6d    &nu,
                              const Eigen::Vector3d    &x_d,
                              const Eigen::Quaterniond &q_d);

  // Return control vector with restoring forces
  Eigen::Vector6d compute(const Eigen::Vector3d    &x,
                          const Eigen::Quaterniond &q,
                          const Eigen::Vector6d    &nu,
                          const Eigen::Vector3d    &x_d,
                          const Eigen::Quaterniond &q_d);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Matrix6d proportionalGainMatrix(const Eigen::Matrix3d &R);
  Eigen::Vector6d errorVector(const Eigen::Vector3d    &p,
                              const Eigen::Vector3d    &p_d,
                              const Eigen::Quaterniond &q,
                              const Eigen::Quaterniond &q_d);
  Eigen::Vector6d restoringForceVector(const Eigen::Matrix3d &R);
  int             sgn(double x);

  double m_c;             // Orientation gain
  Eigen::Matrix6d m_K_d;  // Derivative gain matrix
  Eigen::Matrix3d m_K_x;  // Position error gain matrix

  Eigen::Vector3d m_r_G;  // Center of gravity, expressed in body frame
  Eigen::Vector3d m_r_B;  // Center of buoyancy, expressed in body frame
  double m_W;             // [N] Weight of ROV
  double m_B;             // [N] Buoyancy of ROV
};

#endif  // VORTEX_CONTROLLER_QUATERNION_PD_CONTROLLER_H

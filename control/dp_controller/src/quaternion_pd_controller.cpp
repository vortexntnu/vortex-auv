/*   Written by Kristoffer Rakstad Solberg, Student
     and Morten Fyhn Amundsen, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */


#include "dp_controller/quaternion_pd_controller.h"

QuaternionPdController::QuaternionPdController(double a, double b, double c, double W, double B,
                                               const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B)
: m_r_G(r_G), m_r_B(r_B), m_W(W), m_B(B)
{
  setGains(a, b, c);
}

void QuaternionPdController::setGains(double a, double b, double c)
{
  m_c   = c;
  m_K_d = a * Eigen::MatrixXd::Identity(6, 6);
  m_K_x = b * Eigen::MatrixXd::Identity(3, 3);
}

Eigen::Vector6d QuaternionPdController::getRestoring(const Eigen::Quaterniond &q)
{
  // Rotate from inertial/world to body
  Eigen::Matrix3d R = q.toRotationMatrix();
  return restoringForceVector(R);
}

Eigen::Vector6d QuaternionPdController::getFeedback(const Eigen::Vector3d    &x,
                                                    const Eigen::Quaterniond &q,
                                                    const Eigen::Vector6d    &nu,
                                                    const Eigen::Vector3d    &x_d,
                                                    const Eigen::Quaterniond &q_d)
{
  // Rotate from inertial/world to body
  Eigen::Matrix3d R   = q.toRotationMatrix();
  Eigen::Matrix6d K_p = proportionalGainMatrix(R);
  Eigen::Vector6d z   = errorVector(x, x_d, q, q_d);
  Eigen::Vector6d g   = restoringForceVector(R);
  return (Eigen::Vector6d() << -m_K_d*nu - K_p*z + g).finished();
}


Eigen::Matrix6d QuaternionPdController::proportionalGainMatrix(const Eigen::Matrix3d &R)
{
  return (Eigen::Matrix6d() << R.transpose() * m_K_x,       Eigen::MatrixXd::Zero(3, 3),
                               Eigen::MatrixXd::Zero(3, 3), m_c*Eigen::MatrixXd::Identity(3, 3)).finished();
}

Eigen::Vector6d QuaternionPdController::errorVector(const Eigen::Vector3d    &x,
                                                    const Eigen::Vector3d    &x_d,
                                                    const Eigen::Quaterniond &q,
                                                    const Eigen::Quaterniond &q_d)
{
  Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
  q_tilde.normalize();

  Eigen::Vector3d error_body = x - x_d;

  return (Eigen::Vector6d() << error_body, sgn(q_tilde.w())*q_tilde.vec()).finished();
}

Eigen::Vector6d QuaternionPdController::restoringForceVector(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, m_W);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, -m_B);
  return (Eigen::Vector6d() << f_G + f_B, m_r_G.cross(f_G) + m_r_B.cross(f_B)).finished();
}

int QuaternionPdController::sgn(double x)
{
  if (x < 0)
    return -1;
  return 1;
}
                                

bool QuaternionPdController::circleOfAcceptance(const Eigen::Vector3d   &x,
                                                const Eigen::Vector3d   &x_d)
{
  float R = 1.0;
  float distance, e_squared;

  Eigen::Vector3d e = x_d-x;
  e_squared = pow(e[0],2.0) + pow(e[1], 2.0) + pow(e[2],2.0);
  distance = sqrt(e_squared);
  std::cout << "distance: " << distance << std::endl;

  return (distance < R);
}
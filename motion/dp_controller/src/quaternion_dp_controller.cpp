/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/quaternion_dp_controller.h"
#include <math.h>

QuaternionPIDController::QuaternionPIDController() { // float W, float B,
                                                     // Eigen::Vector3d r_G,
                                                     // Eigen::Vector3d r_B){
  m_B = 23 * 9.81;
  m_W = 24 * 9.81;
  m_r_G = Eigen::Vector3d::Zero();
  m_r_B = Eigen::Vector3d::Zero();
};

int QuaternionPIDController::sgn(double x) {
  if (x < 0)
    return -1;
  return 1;
}

Eigen::Vector6d QuaternionPIDController::errorVector(
    const Eigen::Vector3d &x, const Eigen::Vector3d &x_d,
    const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d) {
  Eigen::Quaterniond q_tilde = q_d.conjugate() * q;
  q_tilde.normalize();

  Eigen::Vector3d error_body = x - x_d;

  return (Eigen::Vector6d() << error_body, sgn(q_tilde.w()) * q_tilde.vec())
      .finished();
}

Eigen::Matrix6d
QuaternionPIDController::proportionalGainMatrix(const Eigen::Matrix3d R) {
  Eigen::Matrix3d m_c = Eigen::Matrix3d::Zero();
  m_c.diagonal() << m_p_gain.segment(3, 3);
  Eigen::Matrix3d m_K_x = Eigen::Matrix3d::Zero();
  m_K_x.diagonal() << m_p_gain.segment(0, 3);
  return (Eigen::Matrix6d() << R.transpose() * m_K_x,
          Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3), m_c)
      .finished();
}

Eigen::Matrix3d skew(Eigen::Vector3d vec) {
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
  skew_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_mat;
}
Eigen::Vector6d QuaternionPIDController::getFeedback(
    const Eigen::Vector3d &x, const Eigen::Quaterniond &q,
    const Eigen::Vector6d &nu, const Eigen::Vector6d &nu_d,
    const Eigen::Vector3d &eta_d_pos, const Eigen::Quaterniond &eta_d_ori) {

  // Rotate from inertial/world to body

  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix6d K_p = proportionalGainMatrix(R);
  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4, 3);
  T << -q.vec().transpose(),
      q.w() * Eigen::Matrix3d::Identity() + skew(q.vec());
  T = 0.5 * T;
  Eigen::MatrixXd J_inv = Eigen::MatrixXd::Zero(6, 7);
  J_inv << R.transpose(), Eigen::MatrixXd::Zero(3, 4), Eigen::Matrix3d::Zero(),
      4 * T.transpose();
  Eigen::Vector6d nu_tilde = Eigen::Vector6d::Zero();
  Eigen::Vector6d remove_ori = Eigen::Vector6d::Zero();
  remove_ori << 1, 1, 1, 0, 0, 0;
  nu_tilde = nu - nu_d.cwiseProduct(remove_ori);

  // Error Vector
  Eigen::Vector6d z = errorVector(x, eta_d_pos, q, eta_d_ori);

  // Integral
  double maxPosGain = 0.5;
  double maxAttGain = 0.05;
  Eigen::Vector6d IntegralAntiWindup = Eigen::Vector6d::Zero();
  IntegralAntiWindup << maxPosGain, maxPosGain, maxPosGain, maxAttGain,
      maxAttGain, maxAttGain;

  integral += m_i_gain * z;
  integral =
      integral.cwiseMin(IntegralAntiWindup).cwiseMax(-IntegralAntiWindup);

  Eigen::Matrix6d m_K_d = Eigen::Matrix6d::Zero();
  m_K_d.diagonal() << m_d_gain;
  Eigen::Vector6d g = QuaternionPIDController::restoringForceVector(R);

  // gain
  Eigen::Vector6d gain = -m_K_d * nu_tilde - K_p * z + g;
  Eigen::Vector6d scale_g = Eigen::Vector6d::Zero();
  scale_g << 0.9, 0.9, 0.9, 0.3, 0.3, 0.3;
  gain = -m_K_d * nu_tilde - K_p * z + g.cwiseProduct(scale_g) - integral;

  //----------
  P_debug = K_p * z;
  I_debug = integral;
  D_debug = m_K_d * nu_tilde;

  //-----------

  // Rounding gain to remove super small values
  int num_decimals = 3;
  gain = (gain * pow(10, num_decimals)).array().round() / pow(10, num_decimals);
  return (Eigen::Vector6d() << gain).finished();
}

Eigen::Vector6d
QuaternionPIDController::restoringForceVector(const Eigen::Matrix3d R) {
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, m_W);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, -m_B);
  Eigen::Vector6d g = Eigen::Vector6d::Zero();
  return (Eigen::Vector6d() << f_G + f_B, m_r_G.cross(f_G) + m_r_B.cross(f_B))
      .finished();
}

void QuaternionPIDController::init(const double W, const double B,
                                   const Eigen::Vector3d &r_G,
                                   const Eigen::Vector3d &r_B) {
  m_W = W;
  m_B = B;
  m_r_G = r_G;
  m_r_B = r_B;
}

void QuaternionPIDController::update_gain(Eigen::Vector6d p_gain,
                                          Eigen::Vector6d i_gain,
                                          Eigen::Vector6d d_gain) {
  m_p_gain = p_gain;
  m_i_gain.diagonal() << i_gain;
  for (int i = 0; i < 6; i++) {
    if (i_gain(i) == 0) {
      integral(i) = 0;
    }
  }
  m_d_gain = d_gain;
}
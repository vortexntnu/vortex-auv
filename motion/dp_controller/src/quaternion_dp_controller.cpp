/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

// TODO: The controller does not work for the 3 rot DOFs, and only functions as
// intended for yaw. Should be looked into in the future.

#include "dp_controller/quaternion_dp_controller.h"
#include <math.h>

// Quaternion to Euler
Eigen::Vector3d
QuaternionPIDController::quaterniondToEuler(Eigen::Quaterniond q) {
  // Compute roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // Compute pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Compute yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Vector3d
QuaternionPIDController::smallestAngle(Eigen::Vector3d euler_angles) {
  Eigen::Vector3d smallest_euler_angles = Eigen::Vector3d::Zero();
  for (int i = 0; i < euler_angles.size(); i++) {
    if (euler_angles(i) > M_PI) {
      smallest_euler_angles(i) = euler_angles(i) - 2 * M_PI;
    } else if (euler_angles(i) < -M_PI) {
      smallest_euler_angles(i) = euler_angles(i) + 2 * M_PI;
    } else {
      smallest_euler_angles(i) = euler_angles(i);
    }
  }

  return smallest_euler_angles;
}

QuaternionPIDController::QuaternionPIDController() { // float W, float B,
                                                     // Eigen::Vector3d r_G,
                                                     // Eigen::Vector3d r_B){
  m_B = 0;
  m_W = 0;
  m_r_G = Eigen::Vector3d::Zero();
  m_r_B = Eigen::Vector3d::Zero();
  m_p_gain = Eigen::Vector6d::Zero();
  m_i_gain = Eigen::Matrix6d::Zero();
  m_d_gain = Eigen::Vector6d::Zero();
  m_integral = Eigen::Vector6d::Zero();

  // TODO: Should be made into ROS params
  double maxPosGain = 3;
  double maxAttGain = 0.2;

  m_integralAntiWindup = Eigen::Vector6d::Zero();
  m_integralAntiWindup << maxPosGain, maxPosGain, maxPosGain, maxAttGain,
      maxAttGain, maxAttGain;

  // The AUV is to stable in orientation, therefore the scaling of 0.3. The
  // reason of 0.9 scaling in position, is beacause the g-vector may not be
  // equal to the real value.
  // TODO: Should be made into ROS params
  m_scale_g = Eigen::Vector6d::Zero();
  m_scale_g << 0, 0, 0.25, 0.0, 0.0, 0.0;
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
  return (Eigen::Vector6d() << error_body,
          tanh(100 * q_tilde.w()) * q_tilde.vec())
      .finished();
  // return (Eigen::Vector6d() << error_body, sgn(q_tilde.w()) * q_tilde.vec())
  //     .finished();
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

  // Integral (TODO:change Antiwindup to a more advanced one)
  m_integral += m_i_gain * z;
  m_integral =
      m_integral.cwiseMin(m_integralAntiWindup).cwiseMax(-m_integralAntiWindup);

  Eigen::Matrix6d m_K_d = Eigen::Matrix6d::Zero();
  m_K_d.diagonal() << m_d_gain;
  Eigen::Vector6d g = QuaternionPIDController::restoringForceVector(R);

  // gain
  Eigen::Vector6d gain = -m_K_d * nu_tilde - K_p * z + g;
  gain = -m_K_d * nu_tilde - K_p * z + g.cwiseProduct(m_scale_g) - m_integral;
  gain = -m_K_d * nu_tilde - K_p * z - m_integral;

  //------ Debug ----
  P_debug = K_p * z;
  I_debug = m_integral;
  D_debug = m_K_d * nu_tilde;
  g_debug = g;
  //-----------------

  // Rounding gain to remove super small values
  int num_decimals = 3;
  gain = (gain * pow(10, num_decimals)).array().round() / pow(10, num_decimals);
  return (Eigen::Vector6d() << gain).finished();
}

//  BACKUP METHOD
Eigen::Vector6d QuaternionPIDController::getFeedback_euler(
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

  Eigen::Vector6d z = Eigen::Vector6d::Zero();
  z << x - eta_d_pos,
      smallestAngle(quaterniondToEuler(q) - quaterniondToEuler(eta_d_ori));

  // Integral (TODO:change Antiwindup to a more advanced one)
  m_integral += m_i_gain * z;
  m_integral =
      m_integral.cwiseMin(m_integralAntiWindup).cwiseMax(-m_integralAntiWindup);

  Eigen::Matrix6d m_K_d = Eigen::Matrix6d::Zero();
  m_K_d.diagonal() << m_d_gain;
  Eigen::Vector6d g = QuaternionPIDController::restoringForceVector(R);

  // gain
  Eigen::Vector6d gain = -m_K_d * nu_tilde - K_p * z + g;
  gain = -m_K_d * nu_tilde - K_p * z + g.cwiseProduct(m_scale_g) - m_integral;
  gain = -m_K_d * nu_tilde - K_p * z - m_integral;

  //------ Debug ----
  P_debug = K_p * z;
  I_debug = m_integral;
  D_debug = m_K_d * nu_tilde;
  g_debug = g;
  //-----------------

  // Rounding gain to remove super small values
  int num_decimals = 3;
  gain = (gain * pow(10, num_decimals)).array().round() / pow(10, num_decimals);
  return (Eigen::Vector6d() << gain).finished();
}

//  BACKUP METHOD
Eigen::Vector6d QuaternionPIDController::getFeedback_euler(
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

  Eigen::Vector6d z = Eigen::Vector6d::Zero();
  z << x - eta_d_pos,
      smallestAngle(quaterniondToEuler(q) - quaterniondToEuler(eta_d_ori));

  // Integral (TODO:change Antiwindup to a more advanced one)
  m_integral += m_i_gain * z;

  if (launch_type == "real") {
    m_integral(1) -= 2 * (m_i_gain * z)(1);
    m_integral(2) -= 2 * (m_i_gain * z)(2);
  }

  m_integral =
      m_integral.cwiseMin(m_integralAntiWindup).cwiseMax(-m_integralAntiWindup);

  Eigen::Matrix6d m_K_d = Eigen::Matrix6d::Zero();
  m_K_d.diagonal() << m_d_gain;
  Eigen::Vector6d g = QuaternionPIDController::restoringForceVector(R);

  if (launch_type == "simulator") {
    m_scale_g << 0, 0, 0.1, 0, 0, 0;
  }
  // gain
  Eigen::Vector6d gain = -m_K_d * nu_tilde - K_p * z + g;
  gain = -m_K_d * nu_tilde - K_p * z + g.cwiseProduct(m_scale_g) - m_integral;

  //------ Debug ----
  P_debug = K_p * z;
  I_debug = m_integral;
  D_debug = m_K_d * nu_tilde;
  g_debug = g.cwiseProduct(m_scale_g);
  //-----------------

  // Rounding gain to remove super small values
  // TODO: Is this the best way to round to a fixed number of decimals?
  // gain.array().round(num_decimals) ?
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

// TODO: Move this code to the ctor.
void QuaternionPIDController::init(const double W, const double B,
                                   const Eigen::Vector3d &r_G,
                                   const Eigen::Vector3d &r_B) {
  m_W = W;
  m_B = B;
  m_r_G = r_G;
  m_r_B = r_B;
}

// TODO: is the for-loop required?
void QuaternionPIDController::update_gain(Eigen::Vector6d p_gain,
                                          Eigen::Vector6d i_gain,
                                          Eigen::Vector6d d_gain) {
  m_p_gain = p_gain;
  m_i_gain.diagonal() << i_gain;
  for (int i = 0; i < 6; i++) {
    if (i_gain(i) == 0) {
      m_integral(i) = 0;
    }
  }
  m_d_gain = d_gain;
}

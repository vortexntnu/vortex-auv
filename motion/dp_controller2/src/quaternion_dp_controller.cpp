//#include "quaternion_dp_controller.h"
#include "dp_controller2/quaternion_dp_controller.h"
#include <math.h>

QuaternionPIDController::QuaternionPIDController(){};

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

Eigen::Matrix6d QuaternionPIDController::proportionalGainMatrix(const Eigen::Matrix3d R) {
  double m_c = 4;
  double b = 6;
  Eigen::Matrix3d m_K_x = b * Eigen::Matrix3d::Identity();
  return (Eigen::Matrix6d() << R.transpose() * m_K_x,
          Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3),
          m_c * Eigen::MatrixXd::Identity(3, 3))
      .finished();
}

Eigen::Matrix3d skew(Eigen::Vector3d vec){
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
  skew_mat << 0, -vec(2), vec(1), 
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
  return skew_mat;
}

Eigen::Vector6d QuaternionPIDController::getFeedback(
    const Eigen::Vector3d &x, const Eigen::Quaterniond &q,
    const Eigen::Vector6d &nu, const Eigen::Matrix7d eta_dot_d,
    const Eigen::Vector3d &x_d,
    const Eigen::Quaterniond &q_d) {

  //Rotate from inertial/world to body

  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix6d K_p = proportionalGainMatrix(R);

  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4,3);
  T << -q.vec().transpose(), 
        q.w()*Eigen::Matrix3d::Identity() + skew(q.vec());
  T = 0.5*T;

  Eigen::MatrixXd J_inv = Eigen::MatrixXd::Zero(6,7);
  J_inv << R.transpose(), Eigen::MatrixXd::Zero(3,4),
           Eigen::Matrix3d::Zero(), 4*T.transpose();

  Eigen::Vector6d nu_tilde = nu - J_inv*eta_dot_d;

  //Eigen::Matrix6d K_i = integralGainMatrix(R);

  // Reference model
  // Eigen::Vector3d x_d_smooth = referenceModel(x, x_d);

  // Error Vector
  Eigen::Vector6d z = errorVector(x, x_d, q, q_d);

  // Integral
  // double maxPoseGain = 4.0;
  // double maxAttGain = 0.05;
  // integral += K_i * z;
  // integralWindUp(integral, maxPoseGain, maxAttGain);
  int a = 2;
  Eigen::Matrix6d m_K_d = a * Eigen::Matrix6d::Identity();

  // gain
  Eigen::Vector6d gain = -m_K_d * nu_tilde - K_p * z;
  
  return (Eigen::Vector6d() << gain).finished();
}

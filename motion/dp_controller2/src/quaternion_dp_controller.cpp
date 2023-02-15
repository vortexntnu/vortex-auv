//#include "quaternion_dp_controller.h"
#include "dp_controller2/quaternion_dp_controller.h"
#include <math.h>

QuaternionPIDController::QuaternionPIDController(){//float W, float B, Eigen::Vector3d r_G, Eigen::Vector3d r_B){
  m_B = 23*9.81;
  m_W = 24*9.81;
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

Eigen::Matrix6d QuaternionPIDController::proportionalGainMatrix(const Eigen::Matrix3d R) {
  Eigen::Matrix3d m_c = Eigen::Matrix3d::Zero();
  m_c.diagonal() << m_p_gain.segment(3,3);
  std::cout << std::endl << "P_gain: " << m_p_gain.segment(0,3) << std::endl;
  Eigen::Matrix3d m_K_x = Eigen::Matrix3d::Zero();
  std::cout << "DEBUG1" << std::endl;
  m_K_x.diagonal() << m_p_gain.segment(0,3);
  std::cout << "DEBUG2" << std::endl;
  return (Eigen::Matrix6d() << R.transpose() * m_K_x,
          Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Zero(3, 3),
      m_c).finished();
          //m_c //* Eigen::MatrixXd::Identity(3, 3))
}

Eigen::Matrix3d skew(Eigen::Vector3d vec){
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
  skew_mat << 0, -vec(2), vec(1), 
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
  return skew_mat;
}
 Eigen::Vector6d QuaternionPIDController::getFeedback(const Eigen::Vector3d &x,
                              const Eigen::Quaterniond &q,
                              const Eigen::Vector6d &nu,
                              const Eigen::Vector7d &eta_dot_d,
                              const Eigen::Vector3d &eta_d_pos,
                              const Eigen::Quaterniond &eta_d_ori){

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
  Eigen::Vector6d nu_tilde = Eigen::Vector6d::Zero();
  nu_tilde = nu - J_inv*eta_dot_d;
  // std::cout << std::endl << "nu:" << std::endl << nu << std::endl;
  // std::cout << std::endl << "J_inv:" << std::endl << J_inv << std::endl;
  // std::cout << std::endl << "eta_dot_d:" << std::endl << eta_dot_d << std::endl;

  //Eigen::Matrix6d K_i = integralGainMatrix(R);

  // Reference model
  // Eigen::Vector3d x_d_smooth = referenceModel(x, x_d);

  // Error Vector
  Eigen::Vector6d z = errorVector(x, eta_d_pos, q, eta_d_ori);
  std::cout << std::endl << "z:" << std::endl << z << std::endl;

  std::cout << "DEBUG3" << std::endl;


  // Integral
  // double maxPoseGain = 4.0;
  // double maxAttGain = 0.05;
  // integral += K_i * z;
  // integralWindUp(integral, maxPoseGain, maxAttGain);

  Eigen::Matrix6d m_K_d = Eigen::Matrix6d::Zero();
  std::cout << "DEBUG4" << std::endl;
  m_K_d.diagonal() << m_d_gain;
  std::cout << "DEBUG5" << std::endl;
  std::cout << std::endl << "D_gain: " << m_d_gain.segment(0,3) << std::endl;
  Eigen::Vector6d g = QuaternionPIDController::restoringForceVector(R);
  // gain
  Eigen::Vector6d gain = -m_K_d * nu_tilde - K_p * z + g;
  // std::cout << std::endl << "m_K_d:" << std::endl << m_K_d << std::endl;
  // std::cout << std::endl << "nu_tilde:" << std::endl << nu_tilde << std::endl;
  // std::cout << std::endl << "K_p:" << std::endl << K_p << std::endl;


  return (Eigen::Vector6d() << gain).finished();
}


Eigen::Vector6d QuaternionPIDController::restoringForceVector(const Eigen::Matrix3d R) {
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, m_W);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, -m_B);
  return (Eigen::Vector6d() << f_G + f_B, m_r_G.cross(f_G) + m_r_B.cross(f_B))
      .finished();
}

void QuaternionPIDController::init(const double W,const double B, const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B){
  m_W = W;
  m_B = B;
  m_r_G = r_G;
  m_r_B = r_B;
}

void QuaternionPIDController::update_gain(Eigen::Vector6d p_gain, Eigen::Vector6d d_gain){
  m_p_gain = p_gain;
  m_d_gain = d_gain;

}
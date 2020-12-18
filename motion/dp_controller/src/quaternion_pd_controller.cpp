/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */


#include "dp_controller/quaternion_pd_controller.h"

QuaternionPdController::QuaternionPdController(double a, double b, double c, double i, double W, double B,
                                               const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B)
: m_r_G(r_G), m_r_B(r_B), m_W(W), m_B(B)
{
  setGains(a, b, c, i);
}

void QuaternionPdController::setGains(double a, double b, double c, double i)
{
  m_c   = c;
  m_c_i = i*0.1; // this should be chosen otherwise
  m_K_d = a * Eigen::MatrixXd::Identity(6, 6);
  m_K_x = b * Eigen::MatrixXd::Identity(3, 3);
  m_K_i = i * Eigen::MatrixXd::Identity(3, 3);

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
  Eigen::Matrix6d K_i = integralGainMatrix(R);

  // Reference model
  Eigen::Vector3d x_d_smooth = referenceModel(x,x_d);

  // Error Vector
  Eigen::Vector6d z   = errorVector(x, x_d_smooth, q, q_d);
  Eigen::Vector6d g   = restoringForceVector(R);


  // Integral
  double maxPoseGain = 4.0;
  double maxAttGain = 0.05;
  integral += K_i*z;
  integralWindUp(integral,maxPoseGain, maxAttGain);
  
  //gain
  Eigen::Vector6d gain = -m_K_d*nu - K_p*z - integral + g;


  return (Eigen::Vector6d() << gain).finished();
}


Eigen::Matrix6d QuaternionPdController::proportionalGainMatrix(const Eigen::Matrix3d R)
{
  return (Eigen::Matrix6d() << R.transpose() * m_K_x,       Eigen::MatrixXd::Zero(3, 3),
                               Eigen::MatrixXd::Zero(3, 3), m_c*Eigen::MatrixXd::Identity(3, 3)).finished();
}

Eigen::Matrix6d QuaternionPdController::integralGainMatrix(const Eigen::Matrix3d R)
{
  return (Eigen::Matrix6d() << R.transpose() * m_K_i,       Eigen::MatrixXd::Zero(3, 3),
                               Eigen::MatrixXd::Zero(3, 3), m_c_i*Eigen::MatrixXd::Identity(3, 3)).finished();
}

void QuaternionPdController::integralWindUp(Eigen::Vector6d &vec, double pose_lim, double att_lim)
{

  for (int i = 0; i < 3; i++){
    if (vec(i) < -pose_lim || vec(i) > pose_lim){
     double k = copysign(pose_lim,vec(i));
     vec[i] = k; 
    }
  }

  for (int i = 3; i < 6; i++){
    if (vec(i) < -att_lim || vec(i) > att_lim){
     double k = copysign(att_lim,vec(i));
     vec[i] = k; 
    }
  }

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

Eigen::Vector6d QuaternionPdController::restoringForceVector(const Eigen::Matrix3d R)
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
                                                const Eigen::Vector3d   &x_d,
                                                      float             R)
{
  //float R = 1.0;
  float distance, e_squared;

  Eigen::Vector3d e = x_d-x;
  e_squared = pow(e[0],2.0) + pow(e[1], 2.0) + pow(e[2],2.0);
  distance = sqrt(e_squared);

  return (distance < R);
}


Eigen::Vector3d QuaternionPdController::referenceModel(const Eigen::Vector3d   &x,
                                                       const Eigen::Vector3d   &x_ref)
{
  
  Eigen::Vector3d x_d;

  Eigen::Vector3d a_x(1,-1.990024937655860,0.990049813123053);
  Eigen::Vector3d b_x(6.218866798092052e-06,1.243773359618410e-05,6.218866798092052e-06);
  x_d = b_x(0) * x_ref + b_x(1) * x_ref_prev + b_x(2) * x_ref_prev_prev - a_x(1) * x_d_prev - a_x(2) * x_d_prev_prev;

  // x_d[k] = x_d[k-1]
  x_ref_prev_prev = x_ref_prev;
  x_ref_prev = x_ref;
  x_d_prev_prev = x_d_prev;
  x_d_prev = x_d;
  return x_d;
}

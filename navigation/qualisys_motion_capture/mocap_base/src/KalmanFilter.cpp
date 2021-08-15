/*
 * Copyright [2015] [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <Eigen/SVD>
#include <mocap_base/KalmanFilter.h>

using namespace std;
using namespace Eigen;

namespace mocap {

KalmanFilter::KalmanFilter():
  attitude         (Quaterniond::Identity()),
  position         (Vector3d::Zero()),
  angular_vel      (Vector3d::Zero()),
  linear_vel       (Vector3d::Zero()),
  state_cov        (Matrix12d::Identity()),
  input_cov        (Matrix12d::Identity()),
  measurement_cov  (Matrix6d::Identity()),
  filter_status    (INIT_POSE),
  last_time_stamp  (0.0),
  msg_interval     (0.01),
  proc_jacob       (Matrix12d::Zero()),
  meas_jacob       (Matrix6_12d::Zero()),
  proc_noise_jacob (Matrix12d::Zero()),
  meas_noise_jacob (Matrix6d::Zero()) {

  // Needs to be changed based on angular velocity
  // and time interval
  proc_jacob = Matrix12d::Identity();
  proc_noise_jacob = Matrix12d::Identity();

  // Do not need to be changed
  meas_jacob.leftCols<6>() = Matrix6d::Identity();
  meas_noise_jacob = Matrix6d::Identity();

  return;
}

bool KalmanFilter::init(const Matrix12d& u_cov,
    const Matrix6d& m_cov, const int& freq) {
  bool is_valid = true;
  JacobiSVD<Matrix12d> u_svd(u_cov);
  JacobiSVD<Matrix6d> m_svd(m_cov);

  Vector12d u_sigmas = u_svd.singularValues();
  Vector6d m_sigmas = m_svd.singularValues();

  if (u_sigmas(11) < 1e-10) {
    is_valid = false;
    ROS_ERROR("Input Cov is close to singular (least singlar value:%f < 1e-7)",
        u_sigmas(11));
  } else {
    input_cov = u_cov;
  }

  if (m_sigmas(5) < 1e-10) {
    is_valid = false;
    ROS_ERROR("Measurement Cov is close to singular (least singlar value:%f < 1e-7)",
        m_sigmas(5));
  } else {
    measurement_cov = m_cov;
  }

  if (freq < 0) {
    is_valid = false;
    ROS_ERROR("Invalid frequency for filter (%d < 0)", freq);
  } else {
    msg_interval = 1.0 / static_cast<double>(freq);
  }

  return is_valid;
}

bool KalmanFilter::prepareInitialCondition(
    const double& curr_time_stamp,
    const Eigen::Quaterniond& m_attitude,
    const Eigen::Vector3d& m_position) {

  switch (filter_status) {
    case INIT_POSE: {
      // Set the current pose
      last_time_stamp = curr_time_stamp;
      attitude = m_attitude;
      position = m_position;
      filter_status = INIT_TWIST;
      // Set the uncertainty of the state
      state_cov = Matrix12d::Identity();
      return true;
    }
    case INIT_TWIST: {
      // Compute the difference between the current
      // pose and last pose
      Quaterniond dq = m_attitude*attitude.inverse();
      AngleAxisd daa(dq);
      Vector3d dr = m_position-position;
      double dt = curr_time_stamp-last_time_stamp;
      //dt = dt > 0 ? dt : msg_interval;
      dt = dt*0.9 + msg_interval*0.1;
      // Set current pose and velocity
      last_time_stamp += dt;
      attitude = m_attitude;
      position = m_position;
      angular_vel = daa.axis()*daa.angle()/dt;
      linear_vel = dr/dt;
      // Set the uncertainty of the state
      state_cov = Matrix12d::Identity();
      filter_status = READY;
      return true;
    }
    case READY:
      return false;
    default:
      return false;
  }
  return false;
}

bool KalmanFilter::isReady() {
  if (filter_status != READY)
    return false;
  else
    return true;
}

void KalmanFilter::reset() {
  filter_status = INIT_POSE;
  return;
}

void KalmanFilter::prediction(const double& curr_time_stamp) {
  // Propogate the actual state
  double dt = curr_time_stamp - last_time_stamp;
  last_time_stamp = curr_time_stamp;

  Vector3d dw = angular_vel * dt;
  Vector3d dr = linear_vel * dt;

  double dangle = dw.norm();
  Vector3d axis = dw / dangle;
  AngleAxisd daa(dangle, axis);
  Quaterniond dq(daa);
  // Velocities are modeled as constants
  attitude = dq * attitude;
  position = dr + position;

  // Propogate the uncertainty of the estimation error
  // TODO: Optimize this part
  proc_jacob(0,  1) =  dw(2);
  proc_jacob(0,  2) = -dw(1);
  proc_jacob(1,  0) = -dw(2);
  proc_jacob(1,  2) =  dw(0);
  proc_jacob(2,  0) =  dw(1);
  proc_jacob(2,  1) = -dw(0);
  proc_jacob(0,  6) =  dt;
  proc_jacob(1,  7) =  dt;
  proc_jacob(2,  8) =  dt;
  proc_jacob(3,  9) =  dt;
  proc_jacob(4, 10) =  dt;
  proc_jacob(5, 11) =  dt;

  //state_cov = proc_jacob*state_cov*proc_jacob.transpose() +
  //  proc_noise_jacob*input_cov*proc_noise_jacob.transpose();
  state_cov = proc_jacob*state_cov*proc_jacob.transpose() + input_cov;

  //cout << "Process: " << endl;
  //cout << "att: " <<
  //  Vector4d(attitude.w(), attitude.x(), attitude.y(), attitude.z()).transpose() << endl;
  //cout << "pos: " << position.transpose() << endl;
  //cout << "ang: " << angular_vel.transpose() << endl;
  //cout << "lin: " << linear_vel.transpose() << endl;
  //cout << "input noise:\n" << input_cov << endl;
  //cout << "proc_jacob:\n" << proc_jacob << endl;
  //cout << "Pk+1|k:\n" << state_cov << endl;

  return;
}

void KalmanFilter::update(const Eigen::Quaterniond& m_attitude,
    const Eigen::Vector3d& m_position) {
  // TODO: Optimize the whole function
  // Compute the residual of the measurement
  Quaterniond re_q = m_attitude * attitude.inverse();
  AngleAxisd re_aa(re_q);
  if (std::abs(re_aa.angle()) > std::abs(2*M_PI-re_aa.angle())) {
    re_aa.angle() = 2*M_PI - re_aa.angle();
    re_aa.axis() = -re_aa.axis();
  }
  Quaterniond re_qs(re_aa);
  Vector3d re_th(re_qs.x()*2.0, re_qs.y()*2.0, re_qs.z()*2.0);
  Vector3d re_r = m_position - position;
  Vector6d re;
  re.head<3>() = re_th;
  re.tail<3>() = re_r;

  // Compute the covariance of the residual
  //Matrix6d S = measurement_cov +
  //  meas_jacob*state_cov*meas_jacob.transpose();
  Matrix6d S = measurement_cov + state_cov.topLeftCorner<6, 6>();

  // Compute the Kalman gain
  //Matrix12_6d K = state_cov*meas_jacob.transpose()*S.inverse();
  Matrix12_6d K = state_cov.leftCols<6>()*S.inverse();

  // Compute the correction of the state
  Vector12d dx = K * re;

  // Update the state based on the correction
  Vector3d dq_vec3 = dx.head<3>() / 2.0;
  Vector4d dq_vec4 = 1.0/sqrt(1.0+dq_vec3.squaredNorm())*
    Vector4d(dq_vec3(0), dq_vec3(1), dq_vec3(2), 1.0);
  Quaterniond dq(dq_vec4(3), dq_vec4(0), dq_vec4(1), dq_vec4(2));

  attitude = dq * attitude;
  position += dx.segment<3>(3);
  angular_vel += dx.segment<3>(6);
  linear_vel += dx.segment<3>(9);

  // Update the uncertainty of the state/error
  state_cov = (Matrix12d::Identity()-K*meas_jacob) * state_cov;

  //cout << "Error: " << endl;
  //cout << "dat: " << Vector4d(re_q.w(), re_q.x(), re_q.y(), re_q.z()).transpose() << endl;
  //cout << "dpo: " << re_r.transpose() << endl;

  //cout << "Innovation: " << endl;
  //cout << "dat: " << Vector4d(dq.w(), dq.x(), dq.y(), dq.z()).transpose() << endl;
  //cout << "dpo: " << dx.segment<3>(3).transpose() << endl;
  //cout << "dan: " << dx.segment<3>(6).transpose() << endl;
  //cout << "dli: " << dx.segment<3>(9).transpose() << endl;

  //cout << "Update: " << endl;
  //cout << "att: " <<
  //  Vector4d(attitude.w(), attitude.x(), attitude.y(), attitude.z()).transpose() << endl;
  //cout << "pos: " << position.transpose() << endl;
  //cout << "ang: " << angular_vel.transpose() << endl;
  //cout << "lin: " << linear_vel.transpose() << endl;
  //cout << "means_jacob:\n" << meas_jacob << endl;
  //cout << "Pk+1|k:\n" << state_cov << endl;
  //cout << "meas_noise:\n" << measurement_cov << endl;
  //cout << endl;

  return;
}
}

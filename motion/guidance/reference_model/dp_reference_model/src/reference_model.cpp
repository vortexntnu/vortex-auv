/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

// TODO: refactor member variables to have "m_" prefix for readability

#include "dp_reference_model/reference_model.h"
#include "dp_reference_model/eigen_typedefs.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

Eigen::Matrix3d skew(Eigen::Vector3d vec) {
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
  skew_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_mat;
}

ReferenceModel::ReferenceModel(ros::NodeHandle nh) : m_nh(nh) {

  std::string odometry_topic;
  std::vector<double> zeta, omega;

  // Import parameters
  getParameters("dp_rm/zeta", zeta);
  getParameters("dp_rm/omega", omega);

  getParameters("/controllers/dp/odometry_topic", odometry_topic);

  std::vector<double> max_vel_buff;
  getParameters("dp_rm/max_vel", max_vel_buff);
  max_vel << max_vel_buff[0], max_vel_buff[1], max_vel_buff[2], max_vel_buff[3],
      max_vel_buff[4], max_vel_buff[5], max_vel_buff[6];
  // Set up dynamic reconfigure server
  dynamic_reconfigure::Server<
      dp_reference_model::DpReferenceModelConfig>::CallbackType f;
  f = boost::bind(&ReferenceModel::cfgCallback, this, _1, _2);
  m_cfg_server.setCallback(f);

  // Set up a service for xref filter toggle
  m_xref_service =
      nh.advertiseService("/dp_reference_model/enable_x_ref_filter",
                          &ReferenceModel::xrefToggleCallback, this);

  // initialize desired eta and eta_dot as zero
  eta_d = Eigen::Vector7d::Zero();
  eta_d(3) = 1; // the real value of quaternions
  eta_dot_d = Eigen::Vector7d::Zero();

  // subs and pubs
  setpoint_sub = nh.subscribe("/dp_data/reference_point", 1,
                              &ReferenceModel::setpointCallback, this);
  reference_pub =
      nh.advertise<nav_msgs::Odometry>("/reference_model/Odometry_d", 1, this);
  m_odometry_sub = m_nh.subscribe(odometry_topic, 1,
                                  &ReferenceModel::odometryCallback, this);

  Delta = Eigen::Matrix7d::Zero();
  Omega = Eigen::Matrix7d::Zero();
  Delta.diagonal() << zeta[1], zeta[2], zeta[3], zeta[4], zeta[5], zeta[6],
      zeta[7];
  Omega.diagonal() << omega[1], omega[2], omega[3], omega[4], omega[5],
      omega[6], omega[7];

  A_d << Eigen::Matrix7d::Zero(), Eigen::Matrix7d::Identity(), -Omega * Omega,
      -2 * Delta * Omega;

  B_d << Eigen::Matrix7d::Zero(), Omega * Omega;
}

// Eigen::Vector14d
void ReferenceModel::calculate_smooth(Eigen::Vector7d x_ref) {
  Eigen::Vector14d x_d;
  x_d << eta_d, eta_dot_d;

  ros::Time current_time = ros::Time::now();
  double time_step = (current_time - last_time)
                         .toSec(); // calculate time difference in seconds
  last_time = current_time;

  Eigen::Vector14d x_dot_d = A_d * x_d + B_d * x_ref;
  x_d = x_d + time_step * x_dot_d;

  if (m_x_ref_filter_enabled) {
    eta_d = x_d.segment(0, 7);
  } else {
    eta_d = x_ref.segment(0, 7);
  }
  eta_dot_d = x_d.segment(7, 7);
  eta_dot_d = eta_dot_d.cwiseMin(max_vel).cwiseMax(-max_vel);

  // Normalizing desired quaternion
  Eigen::Quaterniond quat_d(eta_d(3), eta_d(4), eta_d(5), eta_d(6));
  // Maybe remove this. This is to only use referene model on three of the
  // quaternions and calculate the real value.
  quat_d.w() = sqrt(1 - std::min(1.0, quat_d.vec().squaredNorm()));
  quat_d.normalize();
  Eigen::Vector4d quat_d_vec(quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z());
  eta_d.segment(3, 4) = quat_d_vec;
}

void ReferenceModel::setpointCallback(const geometry_msgs::Pose &setpoint_msg) {

  // parse msg
  Eigen::Vector7d x_ref_buff = Eigen::Vector7d::Zero();
  x_ref_buff << setpoint_msg.position.x, setpoint_msg.position.y,
      setpoint_msg.position.z, setpoint_msg.orientation.w,
      setpoint_msg.orientation.x, setpoint_msg.orientation.y,
      setpoint_msg.orientation.z;
  Eigen::Matrix3d R = orientation.toRotationMatrix();
  Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4, 3);

  T << -orientation.vec().transpose(),
      orientation.w() * Eigen::Matrix3d::Identity() + skew(orientation.vec());
  T = 0.5 * T;

  Eigen::MatrixXd J_inv = Eigen::MatrixXd::Zero(6, 7);
  J_inv << R.transpose(), Eigen::MatrixXd::Zero(3, 4), Eigen::Matrix3d::Zero(),
      4 * T.transpose();

  if (!x_ref.isApprox(x_ref_buff) ||
      (ros::Time::now() - last_time).toSec() > 5) {

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(7, 6);
    J << R, Eigen::Matrix3d::Zero(), Eigen::MatrixXd::Zero(4, 3), T;

    eta_d << position, orientation.w(), orientation.vec();
    eta_dot_d << J * velocity;

    last_time = ros::Time::now();
  }

  x_ref = x_ref_buff;

  // calculate smooth setpoint
  calculate_smooth(x_ref);

  // convert and publish smooth setpoint
  nav_msgs::Odometry odometry_d_msg;
  geometry_msgs::Twist nu_d_msg;

  // Fill up the header
  odometry_d_msg.header.frame_id = "ReferenceFrame";
  odometry_d_msg.child_frame_id = "";
  odometry_d_msg.header.stamp = ros::Time::now();

  Eigen::Quaterniond eta_d_quat(eta_d(3), eta_d(4), eta_d(5), eta_d(6));
  Eigen::Vector6d nu_d = J_inv * eta_dot_d;

  // Transformation of eta_d to odometry-pose-msg and nu_d to odometry-twist-msg
  tf::pointEigenToMsg(eta_d.segment(0, 3), odometry_d_msg.pose.pose.position);
  tf::quaternionEigenToMsg(eta_d_quat, odometry_d_msg.pose.pose.orientation);

  tf::twistEigenToMsg(nu_d, odometry_d_msg.twist.twist);

  reference_pub.publish(odometry_d_msg);
}

template <typename T>
void ReferenceModel::getParameters(std::string param_name, T &param_variable) {
  if (!m_nh.getParam(param_name, param_variable)) {
    ROS_FATAL("Failed to read parameter %s.  Shutting down node..",
              param_name.c_str());
    ros::shutdown();
  }
}

void ReferenceModel::cfgCallback(
    dp_reference_model::DpReferenceModelConfig &config, uint32_t level) {

  // gets the different gains form ReferenceModel.cfg
  Delta = Eigen::Matrix7d::Zero();
  Omega = Eigen::Matrix7d::Zero();
  Delta.diagonal() << config.zeta_1, config.zeta_2, config.zeta_3,
      config.zeta_4, config.zeta_5, config.zeta_6, config.zeta_7;
  Omega.diagonal() << config.omega_1, config.omega_2, config.omega_3,
      config.omega_4, config.omega_5, config.omega_6, config.omega_7;

  A_d << Eigen::Matrix7d::Zero(), Eigen::Matrix7d::Identity(), -Omega * Omega,
      -2 * Delta * Omega;

  B_d << Eigen::Matrix7d::Zero(), Omega * Omega;
}

void ReferenceModel::odometryCallback(const nav_msgs::Odometry &msg) {
  // Convert to eigen for computation
  tf::pointMsgToEigen(msg.pose.pose.position, position);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation);
  tf::twistMsgToEigen(msg.twist.twist, velocity);
}

bool ReferenceModel::xrefToggleCallback(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res) {
  m_x_ref_filter_enabled = req.data;

  // Set the response value if required
  res.success = true;
  res.message = "Service call succeeded";

  return true;
}
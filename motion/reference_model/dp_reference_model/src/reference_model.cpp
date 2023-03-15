/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_reference_model/reference_model.h"
#include "dp_reference_model/eigen_typedefs.h"
#include <eigen3/Eigen/Dense>
#include <iostream>


Eigen::Matrix3d skew(Eigen::Vector3d vec) {
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();
  skew_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_mat;
}

ReferenceModel::ReferenceModel(ros::NodeHandle nh): m_nh(nh) {

   std::string odometry_topic;

  // Import parameters
  ReferenceModel::getParameters("dp_rm/zeta_1", zeta_1);
  ReferenceModel::getParameters("dp_rm/zeta_2", zeta_2);
  ReferenceModel::getParameters("dp_rm/zeta_3", zeta_3);
  ReferenceModel::getParameters("dp_rm/zeta_4", zeta_4);
  ReferenceModel::getParameters("dp_rm/zeta_5", zeta_5);
  ReferenceModel::getParameters("dp_rm/zeta_6", zeta_6);
  ReferenceModel::getParameters("dp_rm/zeta_7", zeta_7);

  ReferenceModel::getParameters("dp_rm/omega_1", omega_1);
  ReferenceModel::getParameters("dp_rm/omega_2", omega_2);
  ReferenceModel::getParameters("dp_rm/omega_3", omega_3);
  ReferenceModel::getParameters("dp_rm/omega_4", omega_4);
  ReferenceModel::getParameters("dp_rm/omega_5", omega_5);
  ReferenceModel::getParameters("dp_rm/omega_6", omega_6);
  ReferenceModel::getParameters("dp_rm/omega_7", omega_7);

  ReferenceModel::getParameters("/controllers/dp/odometry_topic", odometry_topic);


  std::vector<double> max_vel_buff;
  ReferenceModel::getParameters("dp_rm/max_vel", max_vel_buff);
  max_vel << max_vel_buff[0], max_vel_buff[1], max_vel_buff[2], max_vel_buff[3],
      max_vel_buff[4], max_vel_buff[5], max_vel_buff[6];
  // Set up dynamic reconfigure server
  dynamic_reconfigure::Server<
      dp_reference_model::DpReferenceModelConfig>::CallbackType f;
  f = boost::bind(&ReferenceModel::cfgCallback, this, _1, _2);
  m_cfg_server.setCallback(f);

  // initialize desired eta and eta_dot as zero
  eta_d = Eigen::Vector7d::Zero();
  eta_d(3) = 1; // the real value of quaternions
  eta_dot_d = Eigen::Vector7d::Zero();


  // subs and pubs
  setpoint_sub = nh.subscribe("/dp_data/reference_point", 1,
                              &ReferenceModel::setpointCallback, this);
  reference_pub = nh.advertise<geometry_msgs::PoseArray>(
      "/reference_model/output", 1, this);
  m_odometry_sub =
      m_nh.subscribe(odometry_topic, 1, &ReferenceModel::odometryCallback, this);


  Delta = Eigen::Matrix7d::Zero();
  Omega = Eigen::Matrix7d::Zero();
  Delta.diagonal() << zeta_1, zeta_2, zeta_3, zeta_4, zeta_5, zeta_6, zeta_7;
  Omega.diagonal() << omega_1, omega_2, omega_3, omega_4, omega_5, omega_6,
      omega_7;

  A_d << Eigen::Matrix7d::Zero(), Eigen::Matrix7d::Identity(), -Omega * Omega,
      -2 * Delta * Omega;

  B_d << Eigen::Matrix7d::Zero(), Omega * Omega;
}

Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return q;
}

// Eigen::Vector14d
void ReferenceModel::calculate_smooth(Eigen::Vector7d x_ref) {
  Eigen::Vector14d x_d;
  x_d << eta_d, eta_dot_d;

  Eigen::Vector14d x_dot_d = A_d * x_d + B_d * x_ref;
  x_d = x_d + time_step * x_dot_d;
  eta_d = x_d.segment(0, 7);

  eta_dot_d = x_d.segment(7, 7);
  eta_dot_d = eta_dot_d.cwiseMin(max_vel).cwiseMax(-max_vel);

  // Normalizing desired quaternion
  Eigen::Quaterniond quat_d(eta_d(3), eta_d(4), eta_d(5), eta_d(6));
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
  // std::cout << "Hei på deg din gamle kalosj3!" << std::endl;

  if(!x_ref.isApprox(x_ref_buff)){
    
    Eigen::Matrix3d R = orientation.toRotationMatrix();
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4, 3);

    T << -orientation.vec().transpose(),
    orientation.w() * Eigen::Matrix3d::Identity() + skew(orientation.vec());
    T = 0.5 * T;
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(7, 6);
    J << R, Eigen::Matrix3d::Zero(), 
        Eigen::MatrixXd::Zero(4,3), T;

    eta_d << position, orientation.w(), orientation.vec();
    eta_dot_d << J * velocity;

  }
  x_ref = x_ref_buff;
  // std::cout << "Hei på deg din gamle kalosj!3" << std::endl;

  // calculate smooth setpoint
  calculate_smooth(x_ref);

  // convert and publish smooth setpoint
  geometry_msgs::Pose x_d_pose;
  geometry_msgs::Pose x_d_velocity;
  x_d_pose.position.x = eta_d(0);
  x_d_pose.position.y = eta_d(1);
  x_d_pose.position.z = eta_d(2);
  x_d_pose.orientation.w = eta_d(3);
  x_d_pose.orientation.x = eta_d(4);
  x_d_pose.orientation.y = eta_d(5);
  x_d_pose.orientation.z = eta_d(6);

  x_d_velocity.position.x = eta_dot_d(0);
  x_d_velocity.position.y = eta_dot_d(1);
  x_d_velocity.position.z = eta_dot_d(2);
  x_d_velocity.orientation.w = eta_dot_d(3);
  x_d_velocity.orientation.x = eta_dot_d(4);
  x_d_velocity.orientation.y = eta_dot_d(5);
  x_d_velocity.orientation.z = eta_dot_d(6);

  geometry_msgs::PoseArray posearray;
  posearray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
  posearray.header.frame_id =
      "ReferenceFrame"; // frame id in which the array is published

  // push in array (in C++ a vector, in python a list)
  posearray.poses.push_back(x_d_pose);
  posearray.poses.push_back(x_d_velocity);

  reference_pub.publish(posearray);
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
  std::cout << "Hei på deg din gamle kalosj!6" << std::endl;
  // Convert to eigen for computation
  tf::pointMsgToEigen(msg.pose.pose.position, position);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation);
  tf::twistMsgToEigen(msg.twist.twist, velocity);

  std::cout << "Hei på deg din gamle kalosj!7" << std::endl;

}

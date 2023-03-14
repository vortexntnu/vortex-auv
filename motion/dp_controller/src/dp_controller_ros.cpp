/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/dp_controller_ros.h"
#include "dp_controller/dp_action_server.h"
#include <std_msgs/Float32.h>

// Roll pitch and yaw in Radians
// Euler To Quaternion
Eigen::Quaterniond Controller::EulerToQuaterniond(double roll, double pitch,
                                                  double yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  return q;
}

// Quaternion to Euler
Eigen::Vector3d Controller::QuaterniondToEuler(Eigen::Quaterniond q) {
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


template<typename T> 
void Controller::getParameters(std::string param_name, T &param_variable) {
  if (!m_nh.getParam(param_name, param_variable)) {
    ROS_FATAL("Failed to read parameter %s.  Shutting down node..", param_name.c_str());
    ros::shutdown();
  }
}

// void printGroup(const dynamic_reconfigure::Config::ConstPtr& config_msg){
//   std::vector<std::string> param_names;
//   config_msg->getGroupNames("Gains", param_names);
//   std::cout << "param_names: " << std::endl << param_names;
// }
   
Controller::Controller(ros::NodeHandle nh) : m_nh(nh) {
  // Load rosparams
  std::string odometry_topic;
  std::string thrust_topic;

  float W;
  float B;
  std::vector<double> r_G_vec, r_B_vec;
  std::vector<double> p_gain_vec, i_gain_vec, d_gain_vec;

  //import paramteres
  Controller::getParameters("/controllers/dp/odometry_topic", odometry_topic);
  Controller::getParameters("/thrust/thrust_topic",thrust_topic);
  Controller::getParameters("/physical/weight", W);
  Controller::getParameters("/physical/buoyancy",B);
  Controller::getParameters("/physical/center_of_mass", r_G_vec);
  Controller::getParameters("/physical/center_of_buoyancy", r_B_vec);
  Controller::getParameters("/PID/P", p_gain_vec);
  Controller::getParameters("/PID/I", i_gain_vec);
  Controller::getParameters("/PID/D", d_gain_vec);

  Eigen::Vector3d r_G = Eigen::Vector3d(r_G_vec[0], r_G_vec[1], r_G_vec[2]);
  Eigen::Vector3d r_B = Eigen::Vector3d(r_B_vec[0], r_B_vec[1], r_B_vec[2]);

  Eigen::Vector6d p_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d i_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();

  p_gain << p_gain_vec[0], p_gain_vec[1], p_gain_vec[2], p_gain_vec[3],
      p_gain_vec[4], p_gain_vec[5];
  i_gain << i_gain_vec[0], i_gain_vec[1], i_gain_vec[2], i_gain_vec[3], i_gain_vec[4], i_gain_vec[5];
  d_gain << d_gain_vec[0], d_gain_vec[1], d_gain_vec[2], d_gain_vec[3],
      d_gain_vec[4], d_gain_vec[5];

  m_controller.update_gain(p_gain, i_gain, d_gain);

  // Subscribers
  m_odometry_sub =
      m_nh.subscribe(odometry_topic, 1, &Controller::odometryCallback, this);
  ROS_INFO("DP controller initialized");
  m_desiredpoint_sub = m_nh.subscribe("/reference_model/output", 1,
                                      &Controller::desiredPointCallback, this);

  // Publishers
  m_referencepoint_pub =
      m_nh.advertise<geometry_msgs::Pose>("/dp_data/reference_point", 1, this);
  m_wrench_pub = m_nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  m_reference_return_DEBUG_pub =
      m_nh.advertise<std_msgs::Float32>("/dp_data/DEBUG", 1, this);
  m_reference_return_DEBUG2_pub =
      m_nh.advertise<std_msgs::Float32>("/dp_data/DEBUG2", 1, this);
  m_reference_return_q_tilde_print_pub =
      m_nh.advertise<std_msgs::Float32>("/dp_data/q_tilde_print", 1, this);

  // Set up dynamic reconfigure server
  dynamic_reconfigure::Server<dp_controller::DpControllerConfig>::CallbackType f;
  f = boost::bind(&Controller::cfgCallback, this, _1, _2);
  m_cfg_server.setCallback(f);

  eta_d_pos = Eigen::Vector3d::Zero();
  eta_d_ori = Eigen::Quaterniond::Identity();
  eta_d = Eigen::Vector7d::Zero();
  eta_dot_d = Eigen::Vector7d::Zero();

  m_controller.init(W, B, r_G, r_B);
}

void Controller::spin() {
  ros::Rate rate(1);

  DpAction dp_server("DpAction");

  bool was_active = false;
  geometry_msgs::Wrench tau_msg;
  Eigen::Quaterniond x_ref_ori;
  while (ros::ok()) {

    if (dp_server.run_controller) {
      was_active = true;

      tf::quaternionMsgToEigen(dp_server.goal_.x_ref.orientation, x_ref_ori);
      Eigen::Vector6d tau = m_controller.getFeedback(
          position, orientation, velocity, eta_dot_d, eta_d_pos, x_ref_ori);

      Eigen::Vector6d DOF = Eigen::Vector6d::Zero();
      int i = 0; 
      for (int j : dp_server.goal_.DOF) {
        tau(i) *= j;
        DOF(i) = dp_server.goal_.DOF[i];
        i++;
      }
      tf::wrenchEigenToMsg(tau, tau_msg);

      m_wrench_pub.publish(tau_msg);
      m_referencepoint_pub.publish(dp_server.goal_.x_ref); 
    }

    Eigen::Vector3d orientation_euler = QuaterniondToEuler(orientation);
    dp_server.pose << position, orientation_euler;

  // Makes sure DP always ends by sending 0 thrust 
    if (was_active && !dp_server.run_controller) {
      Eigen::VectorXd tau_zero = Eigen::VectorXd::Zero(6, 1);
      tf::wrenchEigenToMsg(tau_zero, tau_msg);
      m_wrench_pub.publish(tau_msg);
      was_active = false;
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void Controller::odometryCallback(const nav_msgs::Odometry &msg) {
  // Convert to eigen for computation
  tf::pointMsgToEigen(msg.pose.pose.position, position);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation);
  tf::twistMsgToEigen(msg.twist.twist, velocity);
}

int sgn(double x) {
  if (x < 0)
    return -1;
  return 1;
}

void Controller::desiredPointCallback(
    const geometry_msgs::PoseArray &desired_msg) {

  Eigen::Vector3d eta_dot_d_pos;
  Eigen::Quaterniond eta_dot_d_ori;

  tf::pointMsgToEigen(desired_msg.poses[0].position, eta_d_pos);
  tf::quaternionMsgToEigen(desired_msg.poses[0].orientation, eta_d_ori);

  Eigen::Vector3d euler_d_buff = QuaterniondToEuler(eta_d_ori);
  eta_d_ori =
      EulerToQuaterniond(euler_d_buff(0), euler_d_buff(1), euler_d_buff(2));

  tf::pointMsgToEigen(desired_msg.poses[1].position, eta_dot_d_pos);
  tf::quaternionMsgToEigen(desired_msg.poses[1].orientation, eta_dot_d_ori);

  eta_d << eta_d_pos, eta_d_ori.w(), eta_d_ori.vec();
  eta_dot_d << eta_dot_d_pos, eta_dot_d_ori.w(), eta_dot_d_ori.vec();
  std_msgs::Float32 debug_msg;
  Eigen::Vector3d Debug_vec = QuaterniondToEuler(eta_d_ori);
  debug_msg.data = Debug_vec(0) * 180 / M_PI;
  m_reference_return_DEBUG_pub.publish(debug_msg);

  Eigen::Quaterniond q_tilde = eta_d_ori.conjugate() * orientation;
  std_msgs::Float32 debug2_msg;
  debug2_msg.data = QuaterniondToEuler(q_tilde)(0) * 180 / M_PI;
  m_reference_return_DEBUG2_pub.publish(debug2_msg);

  std_msgs::Float32 q_tilde_print;
  q_tilde_print.data = sgn(q_tilde.w()) * q_tilde.x();
  m_reference_return_q_tilde_print_pub.publish(q_tilde_print);
}

void Controller::cfgCallback(dp_controller::DpControllerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request P.x: %f", config.P_gain_x);
  ROS_INFO("Reconfigure Request P.y: %f", config.P_gain_y);

  // gets the different gains form Controller.cfg
  Eigen::Vector6d p_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d i_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();
  p_gain << config.P_gain_x, config.P_gain_y, config.P_gain_z, config.P_gain_roll, config.P_gain_pitch, config.P_gain_yaw;
  i_gain << config.I_gain_x, config.I_gain_y, config.I_gain_z, config.I_gain_roll, config.I_gain_pitch, config.I_gain_yaw;
  d_gain << config.D_gain_x, config.D_gain_y, config.D_gain_z, config.D_gain_roll, config.D_gain_pitch, config.D_gain_yaw;
  m_controller.update_gain(p_gain, i_gain, d_gain);
}



/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/dp_controller_ros.h"
#include "dp_controller/dp_action_server.h"
#include <std_msgs/Float32.h>

// Quaternion to Euler
Eigen::Vector3d Controller::quaterniondToEuler(Eigen::Quaterniond q) {
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

template <typename T>
void Controller::getParameters(std::string param_name, T &param_variable) {
  if (!m_nh.getParam(param_name, param_variable)) {
    ROS_FATAL("Failed to read parameter %s.  Shutting down node..",
              param_name.c_str());
    ros::shutdown();
  }
}

Controller::Controller(ros::NodeHandle nh) : m_nh(nh) {
  // Load rosparams
  std::string odometry_topic;
  std::string thrust_topic;

  float W;
  float B;
  std::vector<double> r_G_vec, r_B_vec;
  std::vector<double> p_gain_vec, i_gain_vec, d_gain_vec;

  // import paramteres
  getParameters("/controllers/dp/odometry_topic", odometry_topic);
  getParameters("/thrust/thrust_topic", thrust_topic);
  getParameters("/physical/weight", W);
  getParameters("/physical/buoyancy", B);
  getParameters("/physical/center_of_mass", r_G_vec);
  getParameters("/physical/center_of_buoyancy", r_B_vec);
  getParameters("/PID/P", p_gain_vec);
  getParameters("/PID/I", i_gain_vec);
  getParameters("/PID/D", d_gain_vec);
  getParameters("/PID/Enable", m_enable_PID);
  getParameters("/guidance/dp/rate", m_rate);
  getParameters("/guidance/dp/acceptance_margins", m_acceptance_margins_vec);

  Eigen::Vector3d r_G = Eigen::Vector3d(r_G_vec[0], r_G_vec[1], r_G_vec[2]);
  Eigen::Vector3d r_B = Eigen::Vector3d(r_B_vec[0], r_B_vec[1], r_B_vec[2]);

  Eigen::Vector6d p_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d i_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();

  p_gain << p_gain_vec[0], p_gain_vec[1], p_gain_vec[2], p_gain_vec[3],
      p_gain_vec[4], p_gain_vec[5];
  i_gain << i_gain_vec[0], i_gain_vec[1], i_gain_vec[2], i_gain_vec[3],
      i_gain_vec[4], i_gain_vec[5];
  d_gain << d_gain_vec[0], d_gain_vec[1], d_gain_vec[2], d_gain_vec[3],
      d_gain_vec[4], d_gain_vec[5];

  m_controller.update_gain(p_gain * m_enable_PID[0], i_gain * m_enable_PID[1],
                           d_gain * m_enable_PID[2]);

  float gravity = 9.81;
  m_controller.init(W * gravity, B * gravity, r_G, r_B);

  // Subscribers
  m_odometry_sub =
      m_nh.subscribe(odometry_topic, 1, &Controller::odometryCallback, this);
  ROS_INFO("DP controller initialized");
  m_desiredpoint_sub = m_nh.subscribe("/reference_model/Odometry_d", 1,
                                      &Controller::desiredPointCallback, this);

  // Publishers
  m_referencepoint_pub =
      m_nh.advertise<geometry_msgs::Pose>("/dp_data/reference_point", 1, this);
  m_wrench_pub = m_nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);

  // Set up dynamic reconfigure server
  dynamic_reconfigure::Server<dp_controller::DpControllerConfig>::CallbackType
      f;
  f = boost::bind(&Controller::cfgCallback, this, _1, _2);
  m_cfg_server.setCallback(f);

  m_eta_d_pos = Eigen::Vector3d::Zero();
  m_eta_d_ori = Eigen::Quaterniond::Identity();
  m_nu_d = Eigen::Vector6d::Zero();

  //----------------- DEBUG ------------------
  if (m_debug) {
    m_dp_P_debug_pub = m_nh.advertise<std_msgs::Float64MultiArray>(
        "/dp_data/P_debug", 1, this);
    m_dp_I_debug_pub = m_nh.advertise<std_msgs::Float64MultiArray>(
        "/dp_data/I_debug", 1, this);
    m_dp_D_debug_pub = m_nh.advertise<std_msgs::Float64MultiArray>(
        "/dp_data/D_debug", 1, this);
    m_dp_g_debug_pub = m_nh.advertise<std_msgs::Float64MultiArray>(
        "/dp_data/g_debug", 1, this);

    m_eta_d_deg_pub =
        m_nh.advertise<std_msgs::Float32>("/dp_data/eta_d_deg", 1, this);
    m_q_tilde_pub =
        m_nh.advertise<std_msgs::Float32>("/dp_data/q_tilde_deg", 1, this);
    m_q_tilde_sgn_pub =
        m_nh.advertise<std_msgs::Float32>("/dp_data/q_tilde_sgn_deg", 1, this);
    m_q_tilde_sgn2_pub =
        m_nh.advertise<std_msgs::Float32>("/dp_data/q_tilde_sgn2_deg", 1, this);
  }
  //------------------ END DEBUG -------------
}

void Controller::spin() {
  ros::Rate rate(m_rate);

  DpAction dp_server("DpAction", m_acceptance_margins_vec);

  bool is_active = false;
  geometry_msgs::Wrench tau_msg;
  // Eigen::Quaterniond x_ref_ori;
  while (ros::ok()) {

    getParameters("/DP/Enable", m_enable_dp);
    dp_server.enable = m_enable_dp;

    if (m_enable_dp) {
      is_active = true;

      // tf::quaternionMsgToEigen(dp_server.goal_.x_ref.orientation, x_ref_ori);
      // Eigen::Vector6d tau =
      //     m_controller.getFeedback(m_position, m_orientation, m_velocity,
      //                              m_nu_d, m_eta_d_pos, m_eta_d_ori);

      Eigen::Quaterniond x_ref_ori;
      tf::quaternionMsgToEigen(dp_server.goal_.x_ref.orientation, x_ref_ori);

      Eigen::Vector6d tau =
          m_controller.getFeedback_euler(m_position, m_orientation, m_velocity,
                                         m_nu_d, m_eta_d_pos, x_ref_ori);

      // tau(5) = -tau(5);
      // ROS_INFO("DEBUG");

      // std::cout << "DEBUG1:" << tau(5) << std::endl << -tau(5) << std::endl;
      tau(2) *= -1;

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

      //----------------- DEBUG -------------------
      if (m_debug) {
        std_msgs::Float64MultiArray P_debug_msg;
        std_msgs::Float64MultiArray I_debug_msg;
        std_msgs::Float64MultiArray D_debug_msg;
        std_msgs::Float64MultiArray g_debug_msg;
        for (int i = 0; i < 6; i++) {
          P_debug_msg.data.push_back(m_controller.P_debug(i));
          I_debug_msg.data.push_back(m_controller.I_debug(i));
          D_debug_msg.data.push_back(m_controller.D_debug(i));
          g_debug_msg.data.push_back(m_controller.g_debug(i));
        }

        m_dp_P_debug_pub.publish(P_debug_msg);
        m_dp_I_debug_pub.publish(I_debug_msg);
        m_dp_D_debug_pub.publish(D_debug_msg);
        m_dp_g_debug_pub.publish(g_debug_msg);
      }
      //-------------------- END DEBUG -------------------
    }

    Eigen::Vector3d orientation_euler = quaterniondToEuler(m_orientation);
    dp_server.pose << m_position, orientation_euler;

    // Makes sure DP always ends by sending 0 thrust
    if (is_active && !m_enable_dp) {
      Eigen::VectorXd tau_zero = Eigen::VectorXd::Zero(6, 1);
      tf::wrenchEigenToMsg(tau_zero, tau_msg);
      m_wrench_pub.publish(tau_msg);
      is_active = false;
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void Controller::odometryCallback(const nav_msgs::Odometry &msg) {
  // Convert to eigen for computation
  tf::pointMsgToEigen(msg.pose.pose.position, m_position);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, m_orientation);
  tf::twistMsgToEigen(msg.twist.twist, m_velocity);
}

int sgn(double x) {
  if (x < 0)
    return -1;
  return 1;
}

void Controller::desiredPointCallback(const nav_msgs::Odometry &desired_msg) {

  // Convert to eigen for computation
  tf::pointMsgToEigen(desired_msg.pose.pose.position, m_eta_d_pos);
  tf::quaternionMsgToEigen(desired_msg.pose.pose.orientation, m_eta_d_ori);
  tf::twistMsgToEigen(desired_msg.twist.twist, m_nu_d);

  // --- DEBUG ----------------
  if (m_debug) {
    std_msgs::Float32 eta_d_deg_msg;
    Eigen::Vector3d eta_d_deg_vec = quaterniondToEuler(m_eta_d_ori);
    eta_d_deg_msg.data = eta_d_deg_vec(2) * 180 / M_PI;
    m_eta_d_deg_pub.publish(eta_d_deg_msg);

    Eigen::Quaterniond q_tilde = m_eta_d_ori.conjugate() * m_orientation;
    std_msgs::Float32 q_tilde_msg;
    q_tilde_msg.data = quaterniondToEuler(q_tilde)(2) * 180 / M_PI;
    m_q_tilde_pub.publish(q_tilde_msg);

    std_msgs::Float32 q_tilde_sgn_msg;
    q_tilde_sgn_msg.data = tanh(100 * q_tilde.w()) * q_tilde_msg.data;
    m_q_tilde_sgn_pub.publish(q_tilde_sgn_msg);

    std_msgs::Float32 q_tilde_sgn2_msg;
    q_tilde_sgn2_msg.data = 2 / M_PI * q_tilde.z() * (q_tilde.w() + 1e-6);
    m_q_tilde_sgn2_pub.publish(q_tilde_sgn2_msg);
  }

  // ----- END DEBUG
}

void Controller::cfgCallback(dp_controller::DpControllerConfig &config,
                             uint32_t level) {
  ROS_INFO("Reconfigure Request P.x: %f", config.P_gain_x);
  ROS_INFO("Reconfigure Request P.y: %f", config.P_gain_y);

  // gets the different gains form Controller.cfg
  Eigen::Vector6d p_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d i_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();
  p_gain << config.P_gain_x, config.P_gain_y, config.P_gain_z,
      config.P_gain_roll, config.P_gain_pitch, config.P_gain_yaw;
  i_gain << config.I_gain_x, config.I_gain_y, config.I_gain_z,
      config.I_gain_roll, config.I_gain_pitch, config.I_gain_yaw;
  d_gain << config.D_gain_x, config.D_gain_y, config.D_gain_z,
      config.D_gain_roll, config.D_gain_pitch, config.D_gain_yaw;
  m_controller.update_gain(p_gain * config.P_enable, i_gain * config.I_enable,
                           d_gain * config.D_enable);
}

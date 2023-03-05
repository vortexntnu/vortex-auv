/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
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

Controller::Controller(ros::NodeHandle nh) : m_nh(nh) {
  // Load rosparams
  std::string odometry_topic;
  std::string thrust_topic;

  float W;
  float B;
  std::vector<double> r_G_vec, r_B_vec;
  std::vector<double> p_gain_vec, d_gain_vec;

  if (!nh.getParam("/controllers/dp/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";
  if (!nh.getParam("/thrust/thrust_topic", thrust_topic))
    thrust_topic = "/thrust/desired_forces";

  if (!m_nh.getParam("/physical/weight", W)) {
    ROS_FATAL("Failed to read parameter physical/weight. Shutting down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/physical/buoyancy", B)) {
    ROS_FATAL(
        "Failed to read parameter physical/buoyancy. Shutting down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/physical/center_of_mass", r_G_vec)) {
    ROS_FATAL("Failed to read parameter physical/center_of_mass. Shutting down "
              "node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/physical/center_of_buoyancy", r_B_vec)) {
    ROS_FATAL("Failed to read parameter physical/center_of_buoyancy. Shutting "
              "down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/PID/P", p_gain_vec)) {
    ROS_FATAL("Failed to read parameter PID/P. Shutting down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/PID/D", d_gain_vec)) {
    ROS_FATAL("Failed to read parameter PID/D. Shutting down node..");
    ros::shutdown();
  }

  Eigen::Vector3d r_G = Eigen::Vector3d(r_G_vec[0], r_G_vec[1], r_G_vec[2]);
  Eigen::Vector3d r_B = Eigen::Vector3d(r_B_vec[0], r_B_vec[1], r_B_vec[2]);

  Eigen::Vector6d p_gain = Eigen::Vector6d::Zero();
  Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();

  p_gain << p_gain_vec[0], p_gain_vec[1], p_gain_vec[2], p_gain_vec[3],
      p_gain_vec[4], p_gain_vec[5];
  d_gain << d_gain_vec[0], d_gain_vec[1], d_gain_vec[2], d_gain_vec[3],
      d_gain_vec[4], d_gain_vec[5];

  m_controller.update_gain(p_gain, d_gain);

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

  // m_controller = QuaternionPIDController(2.0);
  //  m_controller = QuaternionPIDController(W, B, r_G, r_B);
  // m_controller()
  eta_d_pos = Eigen::Vector3d::Zero();
  eta_d_ori = Eigen::Quaterniond::Identity();
  eta_d = Eigen::Vector7d::Zero();
  eta_dot_d = Eigen::Vector7d::Zero();

  m_controller.init(W, B, r_G, r_B);
}

void Controller::spin() {
  ros::Rate rate(1);
  Eigen::Vector3d position_setpoint(1, 0, 0);
  Eigen::Quaterniond orientation_setpoint = EulerToQuaterniond(0, 0, 0);

  Eigen::Vector3d position_test(0, 0, 0);
  Eigen::Quaterniond orientation_test = EulerToQuaterniond(0, 0, 0);
  Eigen::Vector6d nu = Eigen::Vector6d::Zero();

  DpAction dp_server("DpAction");

  bool was_active = false;
  geometry_msgs::Wrench tau_msg;
  Eigen::Quaterniond x_ref_ori;
  while (ros::ok()) {
    std::cout << std::endl << "----------Heisann-Hoppsann-----" << std::endl;

    if (dp_server.run_controller) {
      was_active = true;
      std::cout << std::endl << "Heisann-Hoppsann" << std::endl;

      std::cout << "DEBUG6" << std::endl;
      tf::quaternionMsgToEigen(dp_server.goal_.x_ref.orientation, x_ref_ori);
      Eigen::Vector6d tau = m_controller.getFeedback(
          position, orientation, nu, eta_dot_d, eta_d_pos, x_ref_ori);

      std::cout << "Tau:" << std::endl << tau << std::endl;

      std::cout << std::endl << "eta_d_pos:" << eta_d_pos << std::endl;
      std::cout << std::endl
                << "eta_d_ori:" << QuaterniondToEuler(eta_d_ori) << std::endl;
      // position_test = eta_d_pos;
      // orientation_test = eta_d_ori;

      geometry_msgs::Point position_setpoint_msg;
      geometry_msgs::Quaternion orientation_setpoint_msg;
      tf::pointEigenToMsg(position_setpoint, position_setpoint_msg);
      tf::quaternionEigenToMsg(orientation_setpoint, orientation_setpoint_msg);
      geometry_msgs::Pose setpoint_msg;
      setpoint_msg.position = position_setpoint_msg;
      setpoint_msg.orientation = orientation_setpoint_msg;
      // geometry_msgs::Wrench tau_msg;

      Eigen::Vector6d DOF = Eigen::Vector6d::Zero();
      // std::cout << "DOF:" << std::endl << dp_server.goal_.DOF << std::endl;
      int i = 0; //
      std::cout << std::endl
                << "----------------_TAU!_------------" << std::endl;
      for (int j : dp_server.goal_.DOF) {
        // std::cout << "DOF:" << i << std::endl;
        tau(i) *= j;
        std::cout << "tau:" << tau(i) << std::endl;
        DOF(i) = dp_server.goal_.DOF[i];
        i++;
      }
      tf::wrenchEigenToMsg(tau, tau_msg);

      std::cout << std::endl << "----------------------------" << std::endl;
      // std::cout << "DOF:" << std::endl << DOF << std::endl;
      // tau = tau.cwiseProduct(DOF);
      // std::cout << "tau fixed:" << std::endl << tau << std::endl;

      m_wrench_pub.publish(tau_msg);
      m_referencepoint_pub.publish(dp_server.goal_.x_ref); //??????

      // Eigen::Vector3d x_ref_pos;

      // tf::pointMsgToEigen(dp_server.goal_.x_ref.position, x_ref_pos);

      std::vector<double> p_gain_vec, d_gain_vec;

      if (!m_nh.getParam("/PID/P", p_gain_vec)) {
        ROS_FATAL("Failed to read parameter PID/P. Shutting down node..");
        ros::shutdown();
      }
      if (!m_nh.getParam("/PID/D", d_gain_vec)) {
        ROS_FATAL("Failed to read parameter PID/D. Shutting down node..");
        ros::shutdown();
      }

      Eigen::Vector6d p_gain = Eigen::Vector6d::Zero();
      Eigen::Vector6d d_gain = Eigen::Vector6d::Zero();

      p_gain << p_gain_vec[0], p_gain_vec[1], p_gain_vec[2], p_gain_vec[3],
          p_gain_vec[4], p_gain_vec[5];
      d_gain << d_gain_vec[0], d_gain_vec[1], d_gain_vec[2], d_gain_vec[3],
          d_gain_vec[4], d_gain_vec[5];

      std::cout << std::endl << "p_gain: " << p_gain << std::endl;

      m_controller.update_gain(p_gain, d_gain);

      std::cout << "TESSSST5" << std::endl;
    }
    std::cout << std::endl
              << "Orientation:" << std::endl
              << orientation.coeffs() << std::endl;
    std::cout << std::endl << QuaterniondToEuler(orientation) << std::endl;

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
  ROS_DEBUG("position, orientation and velocity states updated");
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

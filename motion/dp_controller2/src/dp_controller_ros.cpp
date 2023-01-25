/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller2/dp_controller_ros.h"
#include "dp_controller2/dp_action_server.h"


//Roll pitch and yaw in Radians
//Euler To Quaternion
Eigen::Quaterniond Controller::EulerToQuaterniond(double roll, double pitch, double yaw){  
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  return q;
}

//Quaternion to Euler
Eigen::Vector3d Controller::QuaterniondToEuler(Eigen::Quaterniond q){
Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
return euler;
}


Controller::Controller(ros::NodeHandle nh) : m_nh(nh) {
    // Load rosparams
  std::string odometry_topic;
  std::string thrust_topic;

  if (!nh.getParam("/controllers/dp/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";
  if (!nh.getParam("/thrust/thrust_topic", thrust_topic))
    thrust_topic = "/thrust/desired_forces";

  // Subscribers
  m_odometry_sub = m_nh.subscribe(odometry_topic, 1, &Controller::odometryCallback, this);
  ROS_INFO("DP controller2 initialized");
  m_desiredpoint_sub = m_nh.subscribe("/reference_model/output", 1, &Controller::desiredPointCallback, this);

  //Publishers
  m_referencepoint_pub = m_nh.advertise<geometry_msgs::Pose>("/dp_data/reference_point", 1, this);
  m_wrench_pub = m_nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  //m_controller = QuaternionPIDController();

  eta_d_pos = Eigen::Vector3d::Zero();
  eta_d_ori = Eigen::Quaterniond::Identity();
  eta_d = Eigen::Vector7d::Zero();
  eta_dot_d = Eigen::Vector7d::Zero();



}


void Controller::spin() {
  ros::Rate rate(1);
  Eigen::Vector3d position_setpoint(1,0,0);
  Eigen::Quaterniond orientation_setpoint = EulerToQuaterniond(0,0,0);

  Eigen::Vector3d position_test(0,0,0);
  Eigen::Quaterniond orientation_test = EulerToQuaterniond(0,0,0);
  Eigen::Vector6d nu = Eigen::Vector6d::Zero();

  DpAction dp_server("DpAction");


  while (ros::ok()) {
    Eigen::Vector6d tau_command = m_controller.getFeedback(
    position_test, orientation_test, nu, eta_dot_d, eta_d_pos,
    eta_d_ori);
    std::cout << "Tau:" << std::endl << tau_command << std::endl;

    position_test = eta_d_pos;
    orientation_test = eta_d_ori;

    geometry_msgs::Point position_setpoint_msg;
    geometry_msgs::Quaternion orientation_setpoint_msg;
    tf::pointEigenToMsg(position_setpoint, position_setpoint_msg);
    tf::quaternionEigenToMsg(orientation_setpoint, orientation_setpoint_msg);
    geometry_msgs::Pose setpoint_msg;
    setpoint_msg.position = position_setpoint_msg;
    setpoint_msg.orientation = orientation_setpoint_msg;
    geometry_msgs::Wrench tau_msg;
    tf::wrenchEigenToMsg(tau_command, tau_msg);
    

    m_wrench_pub.publish(tau_msg);
    m_referencepoint_pub.publish(dp_server.goal_.x_ref);        //??????

    Eigen::Vector3d x_ref_pos;
    Eigen::Quaterniond x_ref_ori;
    tf::pointMsgToEigen(dp_server.goal_.x_ref.position, x_ref_pos);
    tf::quaternionMsgToEigen(dp_server.goal_.x_ref.orientation, x_ref_ori);
    
    
    Eigen::Vector3d orientation_euler = QuaterniondToEuler(orientation_test);       
    dp_server.pose << position_test, orientation_euler;


    std::cout << "TESSSST" << std::endl;
    ros::spinOnce();
    rate.sleep();
  }
}

void Controller::odometryCallback(const nav_msgs::Odometry &msg){
    // Convert to eigen for computation
  tf::pointMsgToEigen(msg.pose.pose.position, position);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation);
  tf::twistMsgToEigen(msg.twist.twist, velocity);
  ROS_DEBUG("position, orientation and velocity states updated");
  
}



void Controller::desiredPointCallback(const geometry_msgs::PoseArray &desired_msg) {

  Eigen::Vector3d eta_dot_d_pos;
  Eigen::Quaterniond eta_dot_d_ori;

  tf::pointMsgToEigen(desired_msg.poses[0].position, eta_d_pos);
  tf::quaternionMsgToEigen(desired_msg.poses[0].orientation, eta_d_ori);

  tf::pointMsgToEigen(desired_msg.poses[0].position, eta_dot_d_pos);
  tf::quaternionMsgToEigen(desired_msg.poses[0].orientation, eta_dot_d_ori);

  eta_d << eta_d_pos, eta_d_ori.w(), eta_d_ori.vec();
  eta_dot_d << eta_dot_d_pos, eta_dot_d_ori.w(), eta_dot_d_ori.vec();

}


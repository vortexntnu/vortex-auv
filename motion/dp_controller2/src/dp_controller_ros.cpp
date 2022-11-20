/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller2/dp_controller_ros.h"


//Roll pitch and yaw in Radians
//Euler To Quaternion
Eigen::Quaterniond Controller::EulerToQuaternion(double roll, double pitch, double yaw){  
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

  if (!nh.getParam("/controllers/dp/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";

  // Subscribers
  m_odometry_sub = m_nh.subscribe(odometry_topic, 1, &Controller::odometryCallback, this);
  ROS_INFO("DP controller2 initialized");

  //m_controller = QuaternionPIDController();



}


void Controller::spin() {
  ros::Rate rate(1);
  Eigen::Vector3d position_setpoint(0,0,0);
  Eigen::Quaterniond orientation_setpoint = EulerToQuaternion(0,0,1);

  Eigen::Vector3d position_test(0,0,0);
  Eigen::Quaterniond orientation_test = EulerToQuaternion(0,0,0);
  Eigen::Vector6d velocity_test = Eigen::Vector6d::Zero();


  while (ros::ok()) {
    Eigen::Vector6d tau = m_controller.getFeedback(
    position_test, orientation_test, velocity_test, position_setpoint,
    orientation_setpoint);
    std::cout << "Tau:" << std::endl << tau << std::endl;
    
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




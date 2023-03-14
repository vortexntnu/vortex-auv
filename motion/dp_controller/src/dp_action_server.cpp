/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/dp_action_server.h"
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <std_msgs/Float32.h>

Eigen::Vector3d SmallestAngle(Eigen::Vector3d euler_angles) {
  Eigen::Vector3d smallest_euler_angles = Eigen::Vector3d::Zero();
  for (int i = 0; i < euler_angles.size(); i++) {
    if (euler_angles(i) > M_PI) {
      smallest_euler_angles(i) = euler_angles(i) - 2 * M_PI;
    } else if (euler_angles(i) < -M_PI) {
      smallest_euler_angles(i) = euler_angles(i) + 2 * M_PI;
    } else {
      smallest_euler_angles(i) = euler_angles(i);
    }
  }

  return smallest_euler_angles;
}

// Quaternion to Euler
Eigen::Vector3d QuaterniondToEuler(Eigen::Quaterniond q) {
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

DpAction::DpAction(std::string name)
    : as_(nh_, name, boost::bind(&DpAction::executeCB, this, _1), false),
      action_name_(name) {
  as_.start();
}

void DpAction::executeCB(const vortex_msgs::dpGoalConstPtr &goal) {
  goal_.x_ref = goal->x_ref;
  goal_.DOF = goal->DOF;
  ros::Rate r(1);
  bool success = false;


  Eigen::Vector6d error = Eigen::Vector6d::Zero();

  //gets the goal-pose from the action_client.
  Eigen::Vector3d x_ref_pos;    
  Eigen::Quaterniond x_ref_ori;
  tf::pointMsgToEigen(goal_.x_ref.position, x_ref_pos);
  tf::quaternionMsgToEigen(goal_.x_ref.orientation, x_ref_ori);

  //gets desired DOF from the action_client.
  Eigen::VectorXd DOF = Eigen::VectorXd::Zero(6, 1);
  for (int i = 0; i < 6; i++) {
    DOF(i) = goal_.DOF[i];
  }
  //Checks if either the goal is cancelled or a new goal is available.
  while (!as_.isPreemptRequested() && ros::ok() && !as_.isNewGoalAvailable()) {
    run_controller = true;
    
    //calculating error.
    feedback_.error.clear();
    Eigen::Vector3d error_pos = x_ref_pos - pose.segment(0, 3);
    Eigen::Vector3d error_ori = QuaterniondToEuler(x_ref_ori) - pose.segment(3, 3);
    error << error_pos, SmallestAngle(error_ori);
    error = DOF.cwiseProduct(error);

    for (int i = 0; i < 6; i++) {
      feedback_.error.push_back(error[i]);
    }

    // publish the feedback
    as_.publishFeedback(feedback_);

    //Checks if the goal is achieved.
    float accepted_radius = 1;      //!!!!!!!!!!!!!!!!!!!!!change
    float accepted_deg = 20;
    double distance_from_goal = error.segment(0, 3).norm();
    Eigen::Vector3d error_ori_deg = error.segment(3, 3) * 180 / M_PI;
    if (distance_from_goal < accepted_radius && abs(error_ori_deg[0]) < accepted_deg &&
        abs(error_ori_deg[1]) < accepted_deg && abs(error_ori_deg[2]) < accepted_deg &&
        success == false) {
      success = true;
      result_.finished = true;
      
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }

    r.sleep();
  }
  
  if(!success) as_.setPreempted();
  run_controller = false;
}
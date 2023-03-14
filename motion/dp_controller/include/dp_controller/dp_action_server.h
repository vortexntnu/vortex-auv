/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#ifndef VORTEX_DP_SERVER_H
#define VORTEX_DP_SERVER_H

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <vortex_msgs/dpAction.h>

#include "eigen_typedefs.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

class DpAction {
private:
  ros::NodeHandle nh_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  vortex_msgs::dpFeedback feedback_;
  vortex_msgs::dpResult result_;

public:
  DpAction(std::string name);

  ~DpAction(void){};

  void executeCB(const vortex_msgs::dpGoalConstPtr &goal);

  Eigen::Vector6d pose;

  vortex_msgs::dpGoal goal_;

  actionlib::SimpleActionServer<vortex_msgs::dpAction>
      as_; // NodeHandle instance must be created before this line. Otherwise
           // strange error occurs.

  bool run_controller = false;
};

#endif // VORTEX_DP_SERVER_H

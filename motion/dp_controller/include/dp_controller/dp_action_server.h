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
  ros::NodeHandle m_nh;
  std::string m_action_name;
  std::vector<double> m_acceptance_margins;

  // create messages that are used to published feedback/result
  vortex_msgs::dpFeedback m_feedback;
  vortex_msgs::dpResult m_result;


public:
  DpAction(std::string name, std::vector<double> acceptance_margins);

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

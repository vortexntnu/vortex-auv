
/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

//#include <math.h>
//#include <cstdlib>
#include <Eigen/Dense>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <dp_controller/VortexControllerConfig.h>
#include "vortex/eigen_typedefs.h"
#include "dp_controller/control_modes.h"
#include "vortex_msgs/PropulsionCommand.h"
#include "vortex_msgs/RovState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "vortex_msgs/Debug.h"
#include "vortex_msgs/ControlMode.h" //service

#include "dp_controller/state.h"
#include "dp_controller/setpoints.h"
#include "dp_controller/quaternion_pd_controller.h"

// Action server
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

// typedef so you dont have to write out definition every time
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class Controller
{

public:
  explicit Controller(ros::NodeHandle nh);

  void stateCallback(const nav_msgs::Odometry &msg);
  void configCallback(const dp_controller::VortexControllerConfig& config, uint32_t level);
  void spin();  
  
  // service
  ros::ServiceServer control_mode_service_;

private:

  // service
  bool controlModeCallback(vortex_msgs::ControlMode::Request  &req,
                           vortex_msgs::ControlMode::Response &res);

  // topics
  ros::NodeHandle m_nh;
  ros::Subscriber m_command_sub;
  ros::Subscriber m_state_sub;
  ros::Subscriber m_mode_sub;
  ros::Publisher  m_wrench_pub;
  ros::Publisher  m_rpm_pub;
  ros::Publisher  m_mode_pub;
  ros::Publisher  m_debug_pub;
  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig> m_dr_srv;

  ControlMode m_control_mode;
  int m_frequency;
  bool m_debug_mode = false;
  const double c_normalized_force_deadzone = 0.01;
  const double c_max_quat_norm_deviation = 0.1;

  enum PoseIndex { SURGE = 0, SWAY = 1, HEAVE = 2, ROLL = 3, PITCH = 4, YAW = 5 };
  enum EulerIndex { EULER_YAW = 0, EULER_PITCH = 1, EULER_ROLL = 2 };

  std::unique_ptr<State>                  m_state;
  std::unique_ptr<Setpoints>              m_setpoints;
  std::unique_ptr<QuaternionPdController> m_controller;

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d    position;
  Eigen::Quaterniond orientation;
  Eigen::Vector6d    velocity;
  Eigen::Vector3d    setpoint_position;
  Eigen::Quaterniond setpoint_orientation;
    // Initial position - hardcoded
  Eigen::Vector3d prev_setpoint_position;;

  ControlMode getControlMode(int mode);
  void initSetpoints();
  void resetSetpoints();
  void updateSetpoint(PoseIndex axis);
  void initPositionHoldController();
  bool healthyMessage(const vortex_msgs::PropulsionCommand &msg);
  void publishControlMode();
  void publishDebugMsg(const Eigen::Vector3d    &position_state,
                       const Eigen::Quaterniond &orientation_state,
                       const Eigen::Vector6d    &velocity_state,
                       const Eigen::Vector3d    &position_setpoint,
                       const Eigen::Quaterniond &orientation_setpoint);
  Eigen::Vector6d stayLevel(const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state);
  
  Eigen::Vector6d depthHold(const Eigen::Vector6d &tau_openloop,
                            const Eigen::Vector3d &position_state,
                            const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state,
                            const Eigen::Vector3d &position_setpoint);

  Eigen::Vector6d headingHold(const Eigen::Vector6d &tau_openloop,
                              const Eigen::Vector3d &position_state,
                              const Eigen::Quaterniond &orientation_state,
                              const Eigen::Vector6d &velocity_state,
                              const Eigen::Quaterniond &orientation_setpoint);

  // NEW BY KRISTOFFER
  Eigen::Vector6d poseHold(const Eigen::Vector6d &tau_openloop,
                            const Eigen::Vector3d &position_state,
                            const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state,
                            const Eigen::Vector3d &position_setpoint,
                            const Eigen::Quaterniond &orientation_setpoint);

  Eigen::Vector6d poseHeadingHold(const Eigen::Vector6d &tau_openloop,
                                  const Eigen::Vector3d &position_state,
                                  const Eigen::Quaterniond &orientation_state,
                                  const Eigen::Vector6d &velocity_state,
                                  const Eigen::Vector3d &position_setpoint,
                                  const Eigen::Quaterniond &orientation_setpoint);

protected:

  // Action object
  MoveBaseActionServer* mActionServer;

  // feedback variable
  move_base_msgs::MoveBaseFeedback feedback_;
  
  // circle of acceptance
  float R;

  // goal variable
  geometry_msgs::PoseStamped mGoal;

public:
  // Action server
  // Called when a new goal is set, simply accepts the goal
  void actionGoalCallBack();

  // Called when e.g. rviz sends us a simple goal.
  void preemptCallBack();

};

#endif  // VORTEX_CONTROLLER_CONTROLLER_ROS_H

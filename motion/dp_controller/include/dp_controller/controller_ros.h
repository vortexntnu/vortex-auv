
/*   Written by Kristoffer Rakstad Solberg, Student
     Documented by Christopher Strøm
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief A ROS wrapper layer for the quaternion PID controller
 * 
*/
#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <dp_controller/VortexControllerConfig.h>
#include "eigen_typedefs.h"
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

/**
 * @brief the Controller class
 * 
 * This class serves as a wrapper for the lower-level controller implementation
 * @see quaternion_pd_controller.h
 * 
*/
class Controller
{

public:

  /**
   * @brief Controller class constructor
   * 
   * @param nh ROS nodehandle
  */
  explicit Controller(ros::NodeHandle nh);

  /**
   * @brief Callback for the state subscriber
  */
  void stateCallback(const nav_msgs::Odometry &msg);

  /**
   * @brief Callback for the dynamic reconfigure server
  */
  void configCallback(const dp_controller::VortexControllerConfig& config, uint32_t level);


  /**
   * @brief Service server callback for setting control mode
   * 
   * @param req   The requested control mode
   * @param res   The server respose to the @p req
   * 
   * @return Always returns true if function execution makes it to
   * the end of the function
  */
  bool controlModeCallback(vortex_msgs::ControlMode::Request  &req,
                           vortex_msgs::ControlMode::Response &res);


  /**
   * @brief Action server; goal
   * 
   * Called when a new goal is set, and simply accepts the new goal.
   * 
  */
  void actionGoalCallBack();


  /**
   * @brief Action server; preemptive goal
   * 
   * Called whenever external applications like rviz sends a simple goal.
  */
  void preemptCallBack();


  /**
   * @brief class wrapper for the usual ros::spin() command
  */
  void spin();  
  
  ros::ServiceServer control_mode_service_; /** Control mode service server */

private:

  ros::NodeHandle m_nh;           /** Nodehandle          */

  ros::Subscriber m_command_sub;  /** Command subscriber  */          
  ros::Subscriber m_state_sub;    /** State subscriber    */
  ros::Subscriber m_mode_sub;     /** Mode subscriber     */

  ros::Publisher  m_wrench_pub;   /** Wrench publisher    */
  ros::Publisher  m_rpm_pub;      /** RPM publisher       */
  ros::Publisher  m_mode_pub;     /** Mode publisher      */
  ros::Publisher  m_debug_pub;    /** Debug publisher     */

  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig> m_dr_srv;  /** dynamic_reconfigure server */

  ControlMode m_control_mode;                       /** Current control mode                        */
  int m_frequency;                                  /** Update frequency for controller (ros rate)  */
  bool m_debug_mode = false;                        /** Bool to run in debug mode                   */
  const double c_normalized_force_deadzone = 0.01;  /** Normalized force deadzone                   */
  const double c_max_quat_norm_deviation = 0.1;     /** Maximum normalized deviation (quaternion)   */

  std::unique_ptr<State>                  m_state;      /** Current states (position, orientation, velocity)  */
  std::unique_ptr<Setpoints>              m_setpoints;  /** Current setpoints (wrench, orientation, velocity) */
  std::unique_ptr<QuaternionPdController> m_controller; /** The Quaternion PID controller object              */

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d    position;              /** Current position      */
  Eigen::Quaterniond orientation;           /** Current orientation   */
  Eigen::Vector6d    velocity;              /** Current velocity      */
  Eigen::Vector3d    setpoint_position;     /** Position setpoint     */
  Eigen::Quaterniond setpoint_orientation;  /** Orientation setpoint  */
  Eigen::Vector3d prev_setpoint_position;   /** Previous setpoint position (initial position hardcoded)*/


  enum PoseIndex { SURGE = 0, SWAY = 1, HEAVE = 2, ROLL = 3, PITCH = 4, YAW = 5 };
  enum EulerIndex { EULER_YAW = 0, EULER_PITCH = 1, EULER_ROLL = 2 };

  /**
   * @brief
   * 
   * @param mode
   * 
   * @return
  */
  ControlMode getControlMode(int mode);


  /**
   * @brief
  */
  void initSetpoints();


  /**
   * @brief
  */
  void resetSetpoints();


  /**
   * @brief
   * 
   * @param axis
  */
  void updateSetpoint(PoseIndex axis);


  /**
   * @brief
  */
  void initPositionHoldController();


  /**
   * @brief
   * 
   * @param msg
   * 
   * @return
  */
  bool healthyMessage(const vortex_msgs::PropulsionCommand &msg);


  /**
   * @brief
  */
  void publishControlMode();


  /**
   * @brief 
   * 
   * @param position_state
   * @param orientation_state
   * @param velocity_state
   * 
   * @param position_setpoint
   * @param orientation_setpoint
  */
  void publishDebugMsg(const Eigen::Vector3d    &position_state,
                       const Eigen::Quaterniond &orientation_state,
                       const Eigen::Vector6d    &velocity_state,
                       const Eigen::Vector3d    &position_setpoint,
                       const Eigen::Quaterniond &orientation_setpoint);

  /**
   * @brief Control mode for staying level
   * 
   * @param orientation_state
   * @param velocity_state
   * 
   * @return
  */
  Eigen::Vector6d stayLevel(const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state);
  

  /**
   * @brief Control mode for keeping constant depth
   * 
   * @param tau_openloop
   * 
   * @param position_state
   * @param orientation_state
   * @param velocity_state
   * 
   * @param position_setpoint
   * 
   * @return
  */
  Eigen::Vector6d depthHold(const Eigen::Vector6d &tau_openloop,
                            const Eigen::Vector3d &position_state,
                            const Eigen::Quaterniond &orientation_state,
                            const Eigen::Vector6d &velocity_state,
                            const Eigen::Vector3d &position_setpoint);


  /**
   * @brief Control mode for keeping a fixed heading
   * 
   * @param tau_openloop
   * 
   * @param position_state
   * @param orientation_state
   * @param velocity_state
   * 
   * @param orientation_setpoint
   * 
   * @return 
  */
  Eigen::Vector6d headingHold(const Eigen::Vector6d &tau_openloop,
                              const Eigen::Vector3d &position_state,
                              const Eigen::Quaterniond &orientation_state,
                              const Eigen::Vector6d &velocity_state,
                              const Eigen::Quaterniond &orientation_setpoint);


  /**
   * @brief Control mode for keeping a fixed pose
   * 
   * @param tau_openloop
   * 
   * @param position_state
   * @param orientation_state
   * @param velocity_state
   * 
   * @param position_setpoint
   * @param orientation_setpoint
   * 
   * @return 
  */
  Eigen::Vector6d poseHold(const Eigen::Vector6d &tau_openloop,
                           const Eigen::Vector3d &position_state,
                           const Eigen::Quaterniond &orientation_state,
                           const Eigen::Vector6d &velocity_state,
                           const Eigen::Vector3d &position_setpoint,
                           const Eigen::Quaterniond &orientation_setpoint);


  /**
   * @brief Control mode for keeping both fixed pose and fixed heading
   *  
   * @param tau_openloop
   * 
   * @param position_state
   * @param orientation_state
   * @param velocity_state
   * 
   * @param position_setpoint
   * @param orientation_setpoint
   * 
   * @return 
  */
  Eigen::Vector6d poseHeadingHold(const Eigen::Vector6d &tau_openloop,
                                  const Eigen::Vector3d &position_state,
                                  const Eigen::Quaterniond &orientation_state,
                                  const Eigen::Vector6d &velocity_state,
                                  const Eigen::Vector3d &position_setpoint,
                                  const Eigen::Quaterniond &orientation_setpoint);

protected:

  MoveBaseActionServer* mActionServer;  /** Action server object */

  move_base_msgs::MoveBaseFeedback feedback_; /** Current feedback value*/
  
  float R;  /** Radius of the circle of acceptance */

  geometry_msgs::PoseStamped mGoal; /** The current goal */

};

#endif  // VORTEX_CONTROLLER_CONTROLLER_ROS_H

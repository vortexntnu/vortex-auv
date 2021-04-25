
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

#include <math.h>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_datatypes.h>
#include <dp_controller/control_modes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "vortex_msgs/Debug.h"
#include "vortex_msgs/ControlMode.h"
#include "vortex_msgs/PropulsionCommand.h"
#include "vortex_msgs/RovState.h"
#include "vortex_msgs/DpSetpoint.h"

#include "dp_controller/quaternion_pd_controller.h"
#include "dp_controller/VortexControllerConfig.h"
#include "dp_controller/state.h"
#include "dp_controller/setpoints.h"
#include "dp_controller/eigen_helper.h"
#include "dp_controller/eigen_typedefs.h"

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
   *
   * If the orientation given in @p msg is invalid, this function returns early.
   * Else it publishes the calculated feedback through the action server.
   *
   * @param msg   A nav_msg::Odometry message containing state data about the AUV.
   */
  void stateCallback(const nav_msgs::Odometry& msg);

  /**
   * @brief Callback for the reference model subscriber
   */
  void guidanceCallback(const vortex_msgs::DpSetpoint& msg);

  /**
   * @brief Callback for the dynamic reconfigure server
   *
   * @param config   A VortexControllerConfig object used to store parameters
   * @param level    Unused integer
   *
   * This function sets the controller gains for the @p config and passes these
   * to the @c setGains() command in the controller.
   */
  void configCallback(const dp_controller::VortexControllerConfig& config, uint32_t level);

  //  /**
  //   * @brief Action server; goal
  //   *
  //   * Called when a new goal is set, and simply accepts the new goal.
  //   *
  //  */
  //  void actionGoalCallBack();
  //
  //
  //  /**
  //   * @brief Action server; preemptive goal
  //   *
  //   * Called whenever external applications like rviz sends a simple goal.
  //  */
  //  void preemptCallBack();

  ros::ServiceServer control_mode_service_; /** Control mode service server */

private:
  ros::NodeHandle m_nh; /** Nodehandle          */

  ros::Subscriber m_command_sub;  /** Command subscriber  */
  ros::Subscriber m_state_sub;    /** State subscriber    */
  ros::Subscriber m_guidance_sub; /** Reference model subscriber*/
  ros::Subscriber m_mode_sub;     /** Mode subscriber     */

  ros::Publisher m_wrench_pub; /** Wrench publisher    */
  ros::Publisher m_rpm_pub;    /** RPM publisher       */
  ros::Publisher m_mode_pub;   /** Mode publisher      */
  ros::Publisher m_debug_pub;  /** Debug publisher     */

  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig> m_dr_srv; /** dynamic_reconfigure server */

  ControlMode prev_control_mode;                   /** Previous control mode                        */
  bool m_debug_mode = false;                       /** Bool to run in debug mode                   */
  const double c_normalized_force_deadzone = 0.01; /** Normalized force deadzone                   */
  const double c_max_quat_norm_deviation = 0.1;    /** Maximum normalized deviation (quaternion)   */

  // Internal state
  Eigen::Vector3d m_current_position;
  Eigen::Quaterniond m_current_orientation;
  Eigen::Vector6d m_current_velocity;

  QuaternionPdController m_controller;

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d position;       /** Current position      */
  Eigen::Quaterniond orientation; /** Current orientation   */
  Eigen::Vector6d velocity;       /** Current velocity      */

  enum PoseIndex
  {
    SURGE = 0,
    SWAY = 1,
    HEAVE = 2,
    ROLL = 3,
    PITCH = 4,
    YAW = 5
  };
  enum EulerIndex
  {
    EULER_YAW = 0,
    EULER_PITCH = 1,
    EULER_ROLL = 2
  };

  /**
   * @brief Convert integer to ControlMode
   *
   * @param mode  An integer that is to be converted to a ControlMode
   *
   * @return The ControlMode corresponding to the given @p mode integer
   * as defined in control_modes.h
   */
  ControlMode getControlMode(int mode);

  /**
   * @brief Initialize setpoints by loading and setting default wrench params
   */
  void initSetpoints();

  /**
   * @brief Reset setpoints by setting them equal to current state
   */
  void resetSetpoints();

  /**
   * @brief Update the current setpoint for a given @p axis
   *
   * @param axis  The selected axis for which the setpoint updates
   */
  void updateSetpoint(PoseIndex axis);

  /**
   * @brief Read parameters and initialize the controller
   *
   * @see quaternion_pd_controller.h
   */
  void initPositionHoldController();

  /**
   * @brief Publish a vortex_msgs Debug message containing current state and setpoint data
   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   * @param orientation_setpoint    A quaternion containing the orientation setpoint
   */
  void publishDebugMsg(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                       const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                       const Eigen::Quaterniond& orientation_setpoint);

  /**
   * @brief Control mode for staying level
   *
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @return  A wrench vector for maintaining a level pose
   */
  Eigen::Vector6d stayLevel(const Eigen::Quaterniond& orientation_state, const Eigen::Vector6d& velocity_state);

  /**
   * @brief Control mode for keeping constant depth
   *   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   *
   * @return  A wrench vector for maintaining constant depth
   */
  Eigen::Vector6d depthHold(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                            const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint);

  /**
   * @brief Control mode for keeping a fixed heading
   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param orientation_setpoint    A quaternion containing the orientation setpoint
   *
   * @return  A wrench vector for maintaining constant heading
   */
  Eigen::Vector6d headingHold(const Eigen::Quaterniond& orientation_state, const Eigen::Vector6d& velocity_state,
                              const Eigen::Quaterniond& orientation_setpoint);

  /**
   * @brief Control mode for keeping a fixed position
   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   * @param orientation_setpoint    A quaternion containing the orientation setpoint
   *
   * @return  A wrench vector for maintaining a fixed position
   */
  Eigen::Vector6d positionHold(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                               const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                               const Eigen::Quaterniond& orientation_setpoint);

  /**
   * @brief Control mode for keeping both fixed position and fixed heading
   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   * @param orientation_setpoint    A quaternion containing the orientation setpoint
   *
   * @return  A wrench vector for maintaining both a fixed position and heading
   */
  Eigen::Vector6d positionHeadingHold(const Eigen::Vector3d& position_state,
                                      const Eigen::Quaterniond& orientation_state,
                                      const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                                      const Eigen::Quaterniond& orientation_setpoint);

  /**
   * @brief Control mode for keeping a fixed pose
   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   * @param orientation_setpoint    A quaternion containing the orientation setpoint
   *
   * @return  A feedback wrench for maintaining a fixed pose
   */
  Eigen::Vector6d poseHold(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                           const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                           const Eigen::Quaterniond& orientation_setpoint);

  /**
   * @brief Control mode for keeping a fixed orientation
   *
   * @param orientation_state A quaternion containing the current orientation
   * @param velocity_state A 6d vector containing the current velocity
   * @param orientation_setpoint A quaternion containing the orientation setpoint
   * @return Eigen::Vector6d
   */
  Eigen::Vector6d orientationHold(const Eigen::Quaterniond& orientation_state, const Eigen::Vector6d& velocity_state,
                                  const Eigen::Quaterniond& orientation_setpoint);

  /**
   * @brief Control mode for keeping orientation and depth
   *
   * @param position_state          A 3d vector containing the current body position
   * @param orientation_state       A quaternion containing the current orientation
   * @param velocity_state          A 6d vector containing the current velocity
   *
   * @param position_setpoint       A 3d vector containing the position setpoint
   * @param orientation_setpoint    A quaternion containing the orientation setpoint
   *
   * @return  A feedback wrench for maintaining a fixed pose
   */
  Eigen::Vector6d orientationDepthHold(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                                      const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                                      const Eigen::Quaterniond& orientation_setpoint);

protected:
  MoveBaseActionServer* mActionServer; /** Action server object */

  move_base_msgs::MoveBaseFeedback feedback_; /** Current feedback value*/

  float R; /** Radius of the circle of acceptance */

  geometry_msgs::PoseStamped mGoal; /** The current goal */
};

#endif  // VORTEX_CONTROLLER_CONTROLLER_ROS_H

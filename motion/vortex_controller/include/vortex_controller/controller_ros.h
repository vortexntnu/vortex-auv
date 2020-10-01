#ifndef VORTEX_CONTROLLER_CONTROLLER_ROS_H
#define VORTEX_CONTROLLER_CONTROLLER_ROS_H

#include <Eigen/Dense>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <vortex_controller/VortexControllerConfig.h>
#include "vortex/eigen_typedefs.h"
#include "vortex_controller/control_modes.h"
#include "vortex_msgs/PropulsionCommand.h"
#include "vortex_msgs/RovState.h"
#include "vortex_msgs/Debug.h"

#include "vortex_controller/state.h"
#include "vortex_controller/setpoints.h"
#include "vortex_controller/quaternion_pd_controller.h"

class Controller
{
public:
  explicit Controller(ros::NodeHandle nh);
  void commandCallback(const vortex_msgs::PropulsionCommand &msg);
  void stateCallback(const vortex_msgs::RovState &msg);
  void configCallback(const vortex_controller::VortexControllerConfig& config, uint32_t level);
  void spin();
private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_command_sub;
  ros::Subscriber m_state_sub;
  ros::Publisher  m_wrench_pub;
  ros::Publisher  m_mode_pub;
  ros::Publisher  m_debug_pub;
  dynamic_reconfigure::Server<vortex_controller::VortexControllerConfig> m_dr_srv;

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

  ControlMode getControlMode(const vortex_msgs::PropulsionCommand &msg) const;
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
};

#endif  // VORTEX_CONTROLLER_CONTROLLER_ROS_H

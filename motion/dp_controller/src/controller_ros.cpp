/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/controller_ros.h"

#include "dp_controller/eigen_helper.h"
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include "std_msgs/String.h"

#include <math.h>
#include <map>
#include <string>
#include <vector>

// TODO: unused parameters that will be left when switching from server to topic
// should be removed from the constructor.
Controller::Controller(ros::NodeHandle nh) : m_nh(nh)
{
  // ROS Parameters
  std::string odometry_topic;
  if (!nh.getParam("/controllers/dp/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";
  std::string thrust_topic;
  if (!nh.getParam("/controllers/dp/thrust_topic", thrust_topic))
    thrust_topic = "/thrust/desired_forces";

  // Subscribers
  m_state_sub = m_nh.subscribe(odometry_topic, 1, &Controller::stateCallback, this);
  m_guidance_sub = m_nh.subscribe("/guidance/dp_data", 1, &Controller::guidanceCallback, this);

  // Publishers
  m_wrench_pub = m_nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  m_mode_pub = m_nh.advertise<std_msgs::String>("controller/mode", 10);
  m_debug_pub = m_nh.advertise<vortex_msgs::Debug>("debug/controlstates", 10);

  // Service callback
  control_mode_service_ = m_nh.advertiseService("controlmode_service", &Controller::controlModeCallback, this);

  // Initial control mode and
  m_control_mode = ControlModes::OPEN_LOOP;
  Eigen::Vector3d startPoint(0, 0, 0);
  position = startPoint;
  orientation = Eigen::Quaterniond::Identity();
  velocity = Eigen::VectorXd::Zero(6);

  // initiate the controller
  initPositionHoldController();

  // Set up a dynamic reconfigure server
  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig>::CallbackType dr_cb;
  dr_cb = boost::bind(&Controller::configCallback, this, _1, _2);
  m_dr_srv.setCallback(dr_cb);
}

/* SERVICE SERVER */

bool Controller::controlModeCallback(vortex_msgs::ControlModeRequest& req, vortex_msgs::ControlModeResponse& res)
{
  try
  {
    ControlMode new_control_mode = getControlMode(req.controlmode);

    if (new_control_mode != m_control_mode)
    {
      m_control_mode = new_control_mode;

      // TO AVOID AGGRESSIVE SWITCHING
      // set current target position to previous position
      m_controller->x_d_prev = position;
      m_controller->x_d_prev_prev = position;
      m_controller->x_ref_prev = position;
      m_controller->x_ref_prev_prev = position;

      // Integral action reset
      m_controller->integral = Eigen::Vector6d::Zero();

      ROS_INFO_STREAM("Changing mode to " << controlModeString(m_control_mode) << ".");
      res.result = "success";
    }
    else
    {
      ROS_INFO_STREAM("Keeping control mode: " << controlModeString(m_control_mode) << ".");
      res.result = "success";
    }
  }
  catch (const std::exception& e)
  {
    res.result = "failed";
    ROS_ERROR("failed to set dp control mode");
  }
  publishControlMode();
  return true;
}

ControlMode Controller::getControlMode(int mode)
{
  ControlMode new_control_mode = m_control_mode;
  return static_cast<ControlMode>(mode);
}

void Controller::stateCallback(const nav_msgs::Odometry& msg)
{
  // Convert to eigen for computation
  Eigen::Vector3d position_tmp;
  Eigen::Quaterniond orientation_tmp;
  Eigen::Vector6d velocity_tmp;
  tf::pointMsgToEigen(msg.pose.pose.position, position_tmp);
  tf::quaternionMsgToEigen(msg.pose.pose.orientation, orientation_tmp);
  tf::twistMsgToEigen(msg.twist.twist, velocity_tmp);

  bool orientation_invalid = (abs(orientation_tmp.norm() - 1) > c_max_quat_norm_deviation);
  if (isFucked(position_tmp) || isFucked(velocity_tmp) || orientation_invalid)
  {
    ROS_WARN_THROTTLE(1, "Invalid state estimate received, ignoring...");
    return;
  }
  else
  {
    // Update local states position, orientation and velocity
    position = position_tmp;
    orientation = orientation_tmp;
    velocity = velocity_tmp;
    ROS_DEBUG("position, orientation and velocity states updated");
  }
}

void Controller::guidanceCallback(const geometry_msgs::Pose& msg)
{
  // Declaration of general forces
  Eigen::Vector6d tau_command = Eigen::VectorXd::Zero(6);

  // gets the newest state as Eigen
  Eigen::Vector3d position_setpoint(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond orientation_setpoint(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

  switch (m_control_mode)
  {
    case ControlModes::OPEN_LOOP:
      // let tau_command stay a zero vector
      break;

    case ControlModes::POSITION_HOLD:
      tau_command = positionHold(position, orientation, velocity, position_setpoint, orientation_setpoint);
      break;

    case ControlModes::DEPTH_HOLD:
      tau_command = depthHold(position, orientation, velocity, position_setpoint);
      break;

    case ControlModes::POSITION_HEADING_HOLD:
      tau_command = positionHeadingHold(position, orientation, velocity, position_setpoint, orientation_setpoint);
      break;

    case ControlModes::HEADING_HOLD:
      tau_command = headingHold(position, orientation, velocity, orientation_setpoint);
      break;

    case ControlModes::POSE_HOLD:
      tau_command = poseHold(position, orientation, velocity, position_setpoint, orientation_setpoint);
      break;

    case ControlModes::DEPTH_HEADING_HOLD:
    {
      Eigen::Vector6d tau_depthhold = depthHold(position, orientation, velocity, position_setpoint);
      Eigen::Vector6d tau_headinghold = headingHold(position, orientation, velocity, orientation_setpoint);
      tau_command = tau_depthhold + tau_headinghold;
      break;
    }

    default:
      ROS_ERROR("Default control mode reached.");
  }

  geometry_msgs::Wrench tau_msg;
  tf::wrenchEigenToMsg(tau_command, tau_msg);
  m_wrench_pub.publish(tau_msg);
}

void Controller::configCallback(const dp_controller::VortexControllerConfig& config, uint32_t level)
{
  ROS_INFO("DP controller reconfigure:");
  ROS_INFO("\t velocity_gain: %2.4f", config.velocity_gain);
  ROS_INFO("\t position_gain: %2.4f", config.position_gain);
  ROS_INFO("\t attitude_gain: %2.4f", config.attitude_gain);
  ROS_INFO("\t integral_gain: %2.4f", config.integral_gain);

  m_controller->setGains(config.velocity_gain, config.position_gain, config.attitude_gain, config.integral_gain);
}

void Controller::initPositionHoldController()
{
  // Read controller gains from parameter server
  double a, b, c, i;
  if (!m_nh.getParam("/controllers/dp/velocity_gain", a))
    ROS_ERROR("Failed to read parameter velocity_gain.");
  if (!m_nh.getParam("/controllers/dp/position_gain", b))
    ROS_ERROR("Failed to read parameter position_gain.");
  if (!m_nh.getParam("/controllers/dp/attitude_gain", c))
    ROS_ERROR("Failed to read parameter attitude_gain.");
  if (!m_nh.getParam("/controllers/dp/integral_gain", i))
    ROS_ERROR("Failed to read parameter integral_gain.");

  // Get physical parameters
  double W;
  double B;
  std::vector<double> r_G_vec, r_B_vec;
  if (!m_nh.getParam("/physical/weight", W))
  {
    ROS_FATAL("Failed to read parameter physical/weight. Shutting down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/physical/buoyancy", B))
  {
    ROS_FATAL("Failed to read parameter physical/buoyancy. Shutting down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/physical/center_of_mass", r_G_vec))
  {
    ROS_FATAL("Failed to read parameter physical/center_of_mass. Shutting down node..");
    ros::shutdown();
  }
  if (!m_nh.getParam("/physical/center_of_buoyancy", r_B_vec))
  {
    ROS_FATAL("Failed to read parameter physical/center_of_buoyancy. Shutting down node..");
    ros::shutdown();
  }
  Eigen::Vector3d r_G = Eigen::Vector3d(r_G_vec[0], r_G_vec[1], r_G_vec[2]) / 1000;  // convert from mm to m
  Eigen::Vector3d r_B = Eigen::Vector3d(r_B_vec[0], r_B_vec[1], r_B_vec[2]) / 1000;

  m_controller.reset(new QuaternionPdController(a, b, c, i, W, B, r_G, r_B));
}

void Controller::publishControlMode()
{
  std::string s = controlModeString(m_control_mode);
  std_msgs::String msg;
  msg.data = s;
  m_mode_pub.publish(msg);
}

void Controller::publishDebugMsg(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                                 const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                                 const Eigen::Quaterniond& orientation_setpoint)
{
  vortex_msgs::Debug dbg_msg;

  // Estimated position
  dbg_msg.state_position.x = position_state[0];
  dbg_msg.state_position.y = position_state[1];
  dbg_msg.state_position.z = position_state[2];

  // Estimated linear velocity
  dbg_msg.state_velocity.linear.x = velocity_state[0];
  dbg_msg.state_velocity.linear.y = velocity_state[1];
  dbg_msg.state_velocity.linear.z = velocity_state[2];

  // Estimated angular velocity
  dbg_msg.state_velocity.angular.x = velocity_state[3];
  dbg_msg.state_velocity.angular.y = velocity_state[4];
  dbg_msg.state_velocity.angular.z = velocity_state[5];

  // Setpoint position
  dbg_msg.setpoint_position.x = position_setpoint[0];
  dbg_msg.setpoint_position.y = position_setpoint[1];
  dbg_msg.setpoint_position.z = position_setpoint[2];

  // Debub state euler orientation
  Eigen::Vector3d dbg_state_orientation = orientation_state.toRotationMatrix().eulerAngles(2, 1, 0);
  Eigen::Vector3d dbg_setpoint_orientation = orientation_setpoint.toRotationMatrix().eulerAngles(2, 1, 0);

  // Debug state orientation
  dbg_msg.state_yaw = dbg_state_orientation[0];
  dbg_msg.state_pitch = dbg_state_orientation[1];
  dbg_msg.state_roll = dbg_state_orientation[2];

  // Debug setpoint euler orientation
  dbg_msg.setpoint_yaw = dbg_setpoint_orientation[0];
  dbg_msg.setpoint_pitch = dbg_setpoint_orientation[1];
  dbg_msg.setpoint_roll = dbg_setpoint_orientation[2];

  // publish
  m_debug_pub.publish(dbg_msg);
}

Eigen::Vector6d Controller::depthHold(const Eigen::Vector3d& position_state,
                                      const Eigen::Quaterniond& orientation_state,
                                      const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint)
{
  Eigen::Vector6d tau = m_controller->getFeedback(position_state, Eigen::Quaterniond::Identity(), velocity_state,
                                                  position_setpoint, Eigen::Quaterniond::Identity());

  // Allow only heave feedback command
  tau(SURGE) = 0;
  tau(SWAY) = 0;
  tau(ROLL) = 0;
  tau(PITCH) = 0;
  tau(YAW) = 0;

  return tau;
}

Eigen::Vector6d Controller::headingHold(const Eigen::Vector3d& position_state,
                                        const Eigen::Quaterniond& orientation_state,
                                        const Eigen::Vector6d& velocity_state,
                                        const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller->getFeedback(Eigen::Vector3d::Zero(), orientation_state, velocity_state,
                                                  Eigen::Vector3d::Zero(), orientation_setpoint);

  // Allow only yaw feedback command
  tau(SURGE) = 0;
  tau(SWAY) = 0;
  tau(HEAVE) = 0;
  tau(ROLL) = 0;
  tau(PITCH) = 0;

  return tau;
}

Eigen::Vector6d Controller::positionHold(const Eigen::Vector3d& position_state,
                                         const Eigen::Quaterniond& orientation_state,
                                         const Eigen::Vector6d& velocity_state,
                                         const Eigen::Vector3d& position_setpoint,
                                         const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller->getFeedback(position_state, orientation_state, velocity_state, position_setpoint,
                                                  orientation_setpoint);

  // Allow only surge,sway,heave feedback command
  tau(ROLL) = 0;
  tau(PITCH) = 0;
  tau(YAW) = 0;

  return tau;
}

Eigen::Vector6d Controller::positionHeadingHold(const Eigen::Vector3d& position_state,
                                                const Eigen::Quaterniond& orientation_state,
                                                const Eigen::Vector6d& velocity_state,
                                                const Eigen::Vector3d& position_setpoint,
                                                const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller->getFeedback(position_state, orientation_state, velocity_state, position_setpoint,
                                                  orientation_setpoint);

  tau(ROLL) = 0;
  tau(PITCH) = 0;

  return tau;
}

Eigen::Vector6d Controller::poseHold(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                                     const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                                     const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller->getFeedback(position_state, orientation_state, velocity_state, position_setpoint,
                                                  orientation_setpoint);
  return tau;
}

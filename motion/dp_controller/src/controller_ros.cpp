/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#include "dp_controller/controller_ros.h"

Controller::Controller(ros::NodeHandle nh) : m_nh(nh)
{
  // Load rosparams
  std::string odometry_topic;
  std::string thrust_topic;
  double a, b, c, i;
  double W;
  double B;
  std::vector<double> r_G_vec, r_B_vec;

  if (!nh.getParam("/controllers/dp/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";
  if (!nh.getParam("/controllers/dp/thrust_topic", thrust_topic))
    thrust_topic = "/thrust/desired_forces";

  if (!m_nh.getParam("/controllers/dp/velocity_gain", a))
  {
    ROS_FATAL("Failed to read parameter velocity_gain.");
    ros::shutdown();
  }
  if (!m_nh.getParam("/controllers/dp/position_gain", b))
  {
    ROS_FATAL("Failed to read parameter position_gain.");
    ros::shutdown();
  }
  if (!m_nh.getParam("/controllers/dp/attitude_gain", c))
  {
    ROS_FATAL("Failed to read parameter attitude_gain.");
    ros::shutdown();
  }
  if (!m_nh.getParam("/controllers/dp/integral_gain", i))
  {
    ROS_FATAL("Failed to read parameter integral_gain.");
    ros::shutdown();
  }
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

  // calculate vectors to CG and CB in meters
  Eigen::Vector3d r_G = Eigen::Vector3d(r_G_vec[0], r_G_vec[1], r_G_vec[2]) / 1000;  // convert from mm to m
  Eigen::Vector3d r_B = Eigen::Vector3d(r_B_vec[0], r_B_vec[1], r_B_vec[2]) / 1000;

  // Initiate variables
  prev_control_mode = ControlModes::OPEN_LOOP;
  Eigen::Vector3d startPoint(0, 0, 0);
  position = startPoint;
  orientation = Eigen::Quaterniond::Identity();
  velocity = Eigen::VectorXd::Zero(6);

  // Set up a dynamic reconfigure server
  dynamic_reconfigure::Server<dp_controller::VortexControllerConfig>::CallbackType dr_cb;
  dr_cb = boost::bind(&Controller::configCallback, this, _1, _2);
  m_dr_srv.setCallback(dr_cb);

  // initiate controller
  m_controller.init(a, b, c, i, W, B, r_G, r_B);

  // Publishers
  m_wrench_pub = m_nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  m_mode_pub = m_nh.advertise<std_msgs::String>("controller/mode", 10);
  m_debug_pub = m_nh.advertise<vortex_msgs::Debug>("debug/controlstates", 10);

  // Subscribers
  m_state_sub = m_nh.subscribe(odometry_topic, 1, &Controller::stateCallback, this);
  m_guidance_sub = m_nh.subscribe("/guidance/dp_data", 1, &Controller::guidanceCallback, this);

  ROS_INFO("DP controller initialized");
}

ControlMode Controller::getControlMode(int mode)
{
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

void Controller::guidanceCallback(const vortex_msgs::DpSetpoint& msg)
{
  // Declaration of general forces
  Eigen::Vector6d tau_command = Eigen::VectorXd::Zero(6);

  // gets the newest state as Eigen
  Eigen::Vector3d position_setpoint(msg.setpoint.position.x, msg.setpoint.position.y, msg.setpoint.position.z);
  Eigen::Quaterniond orientation_setpoint(msg.setpoint.orientation.w, msg.setpoint.orientation.x,
                                          msg.setpoint.orientation.y, msg.setpoint.orientation.z);

  // check control mode
  ControlMode control_mode = getControlMode(msg.control_mode);
  if (control_mode != prev_control_mode)
  {
    prev_control_mode = control_mode;

    // TO AVOID AGGRESSIVE SWITCHING
    // set current target position to previous position
    m_controller.x_d_prev = position;
    m_controller.x_d_prev_prev = position;
    m_controller.x_ref_prev = position;
    m_controller.x_ref_prev_prev = position;

    // Integral action reset
    m_controller.integral = Eigen::Vector6d::Zero();

    ROS_INFO_STREAM("Changing mode to " << controlModeString(control_mode) << ".");
  }

  switch (control_mode)
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

    case ControlModes::DEPTH_HEADING_HOLD: {
      Eigen::Vector6d tau_depthhold = depthHold(position, orientation, velocity, position_setpoint);
      Eigen::Vector6d tau_headinghold = headingHold(position, orientation, velocity, orientation_setpoint);
      tau_command = tau_depthhold + tau_headinghold;
      break;
    }

    case ControlModes::ORIENTATION_HOLD:
      tau_command = orientationHold(orientation, velocity, orientation_setpoint);

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

  m_controller.setGains(config.velocity_gain, config.position_gain, config.attitude_gain, config.integral_gain);
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
  Eigen::Vector6d tau = m_controller.getFeedback(position_state, Eigen::Quaterniond::Identity(), velocity_state,
                                                 position_setpoint, Eigen::Quaterniond::Identity());

  // Allow only heave feedback command
  tau(SURGE) = 0;
  tau(SWAY) = 0;
  tau(ROLL) = 0;
  tau(PITCH) = 0;
  tau(YAW) = 0;

  return tau;
}

Eigen::Vector6d Controller::headingHold(const Eigen::Quaterniond& orientation_state,
                                        const Eigen::Vector6d& velocity_state,
                                        const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller.getFeedback(Eigen::Vector3d::Zero(), orientation_state, velocity_state,
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
  Eigen::Vector6d tau = m_controller.getFeedback(position_state, orientation_state, velocity_state, position_setpoint,
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
  Eigen::Vector6d tau = m_controller.getFeedback(position_state, orientation_state, velocity_state, position_setpoint,
                                                 orientation_setpoint);

  tau(ROLL) = 0;
  tau(PITCH) = 0;

  return tau;
}

Eigen::Vector6d Controller::poseHold(const Eigen::Vector3d& position_state, const Eigen::Quaterniond& orientation_state,
                                     const Eigen::Vector6d& velocity_state, const Eigen::Vector3d& position_setpoint,
                                     const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller.getFeedback(position_state, orientation_state, velocity_state, position_setpoint,
                                                 orientation_setpoint);
  return tau;
}

Eigen::Vector6d Controller::orientationHold(const Eigen::Quaterniond& orientation_state,
                                            const Eigen::Vector6d& velocity_state,
                                            const Eigen::Quaterniond& orientation_setpoint)
{
  Eigen::Vector6d tau = m_controller.getFeedback(Eigen::Vector3d::Zero(), orientation_state, velocity_state,
                                                 Eigen::Vector3d::Zero(), orientation_setpoint);
  return tau;
}

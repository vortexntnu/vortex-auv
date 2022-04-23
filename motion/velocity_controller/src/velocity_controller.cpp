#include "velocity_controller/velocity_controller.h"

VelocityController::VelocityController(ros::NodeHandle nh) : nh(nh)
{
  // get params
  std::string DEFAULT_ODOM_TOPIC = "/odometry/filtered";
  std::string DEFAULT_THRUST_TOPIC = "/thrust/desired_forces";
  std::string DEFAULT_VELOCITY_TOPIC = "/controllers/velocity/desired_velocity";
  getParam("/controllers/velocity_controller/odometry_topic", odometry_topic, DEFAULT_ODOM_TOPIC);
  getParam("/thrust/thrust_topic", thrust_topic, DEFAULT_THRUST_TOPIC);
  getParam("/controllers/velocity_controller/desired_velocity_topic", desired_velocity_topic, DEFAULT_VELOCITY_TOPIC);
  getParam("/controllers/velocity_controller/rate", rate);

  std::vector<double> CB;
  std::vector<double> CG;
  getParam("/physical/weight", drone_weight);
  getParam("/physical/buoyancy", drone_buoyancy);
  getParam("/physical/center_of_buoyancy", CB);
  getParam("/physical/center_of_mass", CG);
  center_of_buoyancy = Eigen::Vector3d(CB[0], CB[1], CB[2]) / 1000;  // convert from mm to m
  center_of_gravity = Eigen::Vector3d(CG[0], CG[1], CG[2]) / 1000;

  std::vector<double> P_gains;
  std::vector<double> I_gains;
  std::vector<double> D_gains;
  std::vector<double> F_gains;
  double integral_windup_limit;
  double setpoint_range;
  double max_output_ramp_rate;
  Eigen::Vector6d desired_velocity;
  controller_active = false;
  getParam("/controllers/velocity_controller/P_gains", P_gains);
  getParam("/controllers/velocity_controller/I_gains", I_gains);
  getParam("/controllers/velocity_controller/D_gains", D_gains);
  getParam("/controllers/velocity_controller/F_gains", F_gains);
  getParam("/controllers/velocity_controller/integral_windup_limit", integral_windup_limit);
  getParam("/controllers/velocity_controller/setpoint_range", setpoint_range);
  getParam("/controllers/velocity_controller/max_output_ramp_rate", max_output_ramp_rate);

  // initialize PIDs
  for (int i = 0; i < 6; i++)
  {
    pids.push_back(std::make_unique<MiniPID>(P_gains[i], I_gains[i], D_gains[i], F_gains[i]));
    pids[i]->setMaxIOutput(integral_windup_limit);
    pids[i]->setSetpointRange(setpoint_range);
    pids[i]->setOutputRampRate(max_output_ramp_rate);

    ROS_DEBUG_STREAM("pid" << i << " initialized with: " << P_gains[i] << " " << I_gains[i] << " " << D_gains[i] << " "
                           << F_gains[i]);
    ROS_DEBUG_STREAM("pid_" << i  << " address: " << &pids[i]);
  }
  ROS_DEBUG_STREAM("integral_windup_limit: " << integral_windup_limit);
  ROS_DEBUG_STREAM("setpoint_range: " << setpoint_range);
  ROS_DEBUG_STREAM("max_output_ramp_rate: " << max_output_ramp_rate);

  // create subscribers and publsihers
  odom_recieved = false;
  thrust_pub = nh.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  odom_sub = nh.subscribe(odometry_topic, 1, &VelocityController::odometryCallback, this);

  // create services
  set_velocity_service = nh.advertiseService(desired_velocity_topic, &VelocityController::setVelocity, this);
  reset_service = nh.advertiseService("reset_pid", &VelocityController::resetPidCallback, this);
  set_gains_service =
      nh.advertiseService("set_gains", &VelocityController::setGainsCallback, this);

  // wait for first odometry message
  ROS_INFO("Waiting for odometry message..");

}

void VelocityController::odometryCallback(const nav_msgs::Odometry& odom_msg)
{
  
  if (!odom_recieved)
  {
    odom_recieved = true;
    ROS_INFO("Odometry message recieved");
    ROS_INFO("Velocity controller setup complete");
  }
  tf::twistMsgToEigen(odom_msg.twist.twist, velocity);
  orientation = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
                                   odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
}

bool VelocityController::setVelocity(vortex_msgs::SetVelocityRequest& req, vortex_msgs::SetVelocityResponse& res)
{
  controller_active = req.active;
  if (!controller_active){
    desired_velocity << 0,0,0,0,0,0;
  } else tf::twistMsgToEigen(req.desired_velocity, desired_velocity);
  // transform desired velocity to eigen
  publishControlForces();
  return true;
}

void VelocityController::publishControlForces()
{
  if (!odom_recieved)
  {
    ROS_ERROR("Controller cannot perform since no odometry has been recieved. Ignoring callback.");
  }
  else
  {
    // calculate tau 
    Eigen::Vector6d tau;
    for (int i = 0; i < 6; i++)
    {
      if (desired_velocity[i] != 0)
        tau[i] = pids[i]->getOutput(velocity[i], desired_velocity[i]);
      else
        tau[i] = 0;
    }

    // publish tau as wrench
    geometry_msgs::Wrench thrust_msg;
    tf::wrenchEigenToMsg(tau, thrust_msg);
    thrust_pub.publish(thrust_msg);
  }
}

bool VelocityController::resetPidCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response)
{
  for (int i = 0; i < 6; i++)
    pids[i]->reset();

  ROS_INFO("PIDs have been reset");
  return true;
}

bool VelocityController::setGainsCallback(vortex_msgs::SetPidGainsRequest& request,
                                          vortex_msgs::SetPidGainsResponse& response)
{
  for (int i = 0; i < 6; i++)
  {
    pids[i]->setP(request.P_gains[i]);
    pids[i]->setI(request.I_gains[i]);
    pids[i]->setD(request.D_gains[i]);
    pids[i]->setF(request.F_gains[i]);
    pids[i]->setMaxIOutput(request.integral_windup_limit);
    pids[i]->setSetpointRange(request.setpoint_range);
    pids[i]->setOutputRampRate(request.max_output_ramp_rate);
    pids[i]->reset();
  }
  ROS_INFO("PID reset and gains set to new values");
  return true;
}

Eigen::Vector6d VelocityController::restoringForces()
{
  // calculates restoring forces in ENU (not NED)
  Eigen::Matrix3d R = orientation.toRotationMatrix();
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, -drone_weight);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, drone_buoyancy);
  return (Eigen::Vector6d() << f_G + f_B, center_of_gravity.cross(f_G) + center_of_buoyancy.cross(f_B)).finished();
}

template <typename T>
void VelocityController::getParam(std::string name, T& variable)
{
  if (!nh.getParam(name, variable))
  {
    ROS_FATAL_STREAM("Missing parameter " << name << ". Shutting down node..");
    ros::shutdown();
  }
}

template <typename T>
void VelocityController::getParam(std::string name, T& variable, T& default_value)
{
  if (!nh.getParam(name, variable))
    variable = default_value;
  ROS_DEBUG_STREAM(name << ": " << variable);
}

void VelocityController::spin()
{
  ros::Rate ros_rate(rate);
  while (ros::ok())
  {

    if (controller_active) {
      publishControlForces();
    }
    ros::spinOnce();
    ros_rate.sleep(); 
  }
}

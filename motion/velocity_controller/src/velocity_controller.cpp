#include "velocity_controller/velocity_controller.h"

VelocityController::VelocityController(ros::NodeHandle ros_node)
{
  this->ros_node = ros_node;

  // get params
  std::vector<double> P_gains;
  std::vector<double> I_gains;
  std::vector<double> D_gains;
  std::vector<double> F_gains;
  double integral_windup_limit;
  std::vector<double> CB;
  std::vector<double> CG;

  getParam("/velocity_controller/odometry_topic", odometry_topic, DEFAULT_ODOM_TOPIC);
  getParam("/velocity_controller/thrust_topic", thrust_topic, DEFAULT_THRUST_TOPIC);
  getParam("/velocity_controller/desired_velocity_topic", desired_velocity_topic, DEFAULT_VELOCITY_TOPIC);

  getParam("physical/weight", drone_weight);
  getParam("physical/bouyancy", drone_bouyancy);
  getParam("physical/center_of_bouyancy", CB);
  getParam("physical/center_of_mass", CG);
  center_of_bouyancy = Eigen::Vector3d(CB[0], CB[1], CB[2]);
  center_of_gravity = Eigen::Vector3d(CG[0], CG[1], CG[2]);

  getParam("controllers/velocity_controller/P_gains", P_gains);
  getParam("controllers/velocity_controller/I_gains", I_gains);
  getParam("controllers/velocity_controller/D_gains", D_gains);
  getParam("controllers/velocity_controller/F_gains", F_gains);
  getParam("controllers/velocity_controller/integral_windup_limit", integral_windup_limit);

  // initialize PIDs
  for (int i = 0; i < 6; i++)
  {
    pid[i] = MiniPID(P_gains[i], I_gains[i], D_gains[i], F_gains[i]);
    pid[i].setMaxIOutput(integral_windup_limit);
  }

  // create subscribers and publsihers
  thrust_pub = ros_node.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  odom_sub = ros_node.subscribe(odometry_topic, 1, &VelocityController::odometryCallback, this);
  vel_sub = ros_node.subscribe(desired_velocity_topic, 1, &VelocityController::controlLawCallback, this);

  // wait for first odometry message
  if (!ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic, ros_node, ros::Duration(30)))
  {
    ROS_FATAL("No odometry recieved within initial 30 seconds. Shutting down node..");
    ros_node.shutdown();
  }
}

void VelocityController::odometryCallback(const nav_msgs::Odometry& odom_msg)
{
  // TODO: might have to make this thread safe
  tf::twistMsgToEigen(odom_msg.twist.twist, velocity);
  orientation = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
                                   odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
}

void VelocityController::controlLawCallback(const geometry_msgs::Twist& twist_msg)
{
  // copy velocity to remove chance of odom beeing updated during function call

  Eigen::Vector6d desired_velocity;
  tf::twistMsgToEigen(twist_msg, desired_velocity);

  // calculate restoring forces
  Eigen::Vector6d restoring_forces = restoringForces();

  // calculate tau using MiniPID and restoring forces
  Eigen::Vector6d tau;
  for (int i = 0; i < 6; i++)
  {
    tau[i] = pid[i].getOutput(velocity[i], desired_velocity[i]) - restoring_forces[i];
  }

  // publish tau as wrench
  geometry_msgs::Wrench thrust_msg;
  tf::wrenchEigenToMsg(tau, thrust_msg);
  thrust_pub.publish(thrust_msg);
}

Eigen::Vector6d VelocityController::restoringForces()
{
  // calculates restoring forces in ENU (not NED)
  Eigen::Matrix3d R = orientation.toRotationMatrix();
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, -drone_weight);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, drone_bouyancy);
  return (Eigen::Vector6d() << f_G + f_B, center_of_gravity.cross(f_G) + center_of_bouyancy.cross(f_B)).finished();
}

template <typename T>
void VelocityController::getParam(std::string name, T& variable)
{
  if (!ros_node.getParam(name, variable))
  {
    std::string log_message = "Missing parameter " + name + ". Shutting down node..";
    ROS_FATAL("test");
    ros_node.shutdown();
  }
}

template <typename T>
void VelocityController::getParam(std::string name, T& variable, T default_value)
{
  if (!ros_node.getParam(name, variable))
    variable = default_value;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle ros_node;

  VelocityController velocity_controller(ros_node);

  ros::spin();
}
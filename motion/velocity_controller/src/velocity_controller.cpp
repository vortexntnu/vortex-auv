#include "velocity_controller/velocity_controller.h"


VelocityController::VelocityController(ros::NodeHandle ros_node)
{
  // get params
  if (!ros_node.getParam("/velocity_controller/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";

  if (!ros_node.getParam("/velocity_controller/thrust_topic", thrust_topic))
    thrust_topic = "/thrust/desired";

  if (!ros_node.getParam("velocity_controller/desired_velocity_topic", desired_velocity_topic))
    desired_velocity_topic = "/controller/desired_velocity";

  // create subscribers and publsihers
  // thrust_pub = ros_node.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  // odom_sub = ros_node.subscribe(odometry_topic, 1, &VelocityController::odometryCallback, this);
  // vel_sub = ros_node.subscribe(desired_velocity_topic, 1, &VelocityController::controlLawCallback, this);

  // wait for first odometry message
  if (!ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic, ros_node, ros::Duration(30)))
  {
    ROS_FATAL("No odometry recieved within initial 30 seconds. Shutting down node..");
    ros_node.shutdown();
  }
}

void VelocityController::odometryCallback(const nav_msgs::Odometry& odom_msg)
{
  odometry = odom_msg;  // TODO: might have to make this thread safe
}

void VelocityController::controlLawCallback(const geometry_msgs::Twist& twist_msg)
{
  // copy odometry to remove chance of odom beeing updated during function call
  nav_msgs::Odometry current_odom = odometry;

  // convert to Eigen? Has to be set up, but would simplify everything else

  // calculate restoring forces

  // calculate tau using MiniPID and restoring forces 
  geometry_msgs::Wrench tau;
  

  // publish tau as thrust
  thrust_pub.publish(tau);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle ros_node;

  VelocityController velocity_controller(ros_node);

  ros::spin();
}
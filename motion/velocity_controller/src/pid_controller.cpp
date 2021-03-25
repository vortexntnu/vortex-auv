#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"

class VelocityController
{
public:
  VelocityController(ros::NodeHandle ros_node);
  void odometryCallback(nav_msgs::Odometry& odom_msg);
  void desiredVelocityCallback(geometry_msgs::Twist& twist_msg);

private:
  nav_msgs::Odometry odometry;
};

VelocityController::VelocityController(ros::NodeHandle ros_node)
{
  // get params
  std::string odometry_topic;
  if (!ros_node.getParam("/velocity_controller/odometry_topic", odometry_topic))
    odometry_topic = "/odometry/filtered";

  std::string thrust_topic;
  if (!ros_node.getParam("/velocity_controller/thrust_topic", thrust_topic))
    thrust_topic = "/thrust/desired";

  std::string desired_velocity_topic;
  if (!ros_node.getParam("velocity_controller/desired_velocity_topic", desired_velocity_topic))
    desired_velocity_topic = "/controller/desired_velocity";

  // create subscribers and publsihers
  ros::Publisher thrust_pub = ros_node.advertise<geometry_msgs::Wrench>(thrust_topic, 1);
  ros::Subscriber odom_sub =
      ros_node.subscribe<nav_msgs::Odometry>(odometry_topic, 1, VelocityController::odometryCallback);
  ros::Subscriber vel_sub =
      ros_node.subscribe<geometry_msgs::Twist>(desired_velocity_topic, 1, VelocityController::desiredVelocityCallback);

  // wait for first odometry message
  if (!ros::topic::waitForMessage<nav_msgs::Odometry>(odometry_topic, ros_node, ros::Duration(30)))
  {
    ROS_FATAL("No odometry recieved within initial 30 seconds. Shutting down node..");
    ros_node.shutdown();
  }
}

void VelocityController::odometryCallback(nav_msgs::Odometry& odom_msg)
{
  odometry = odom_msg; // TODO: might have to make this thread safe
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle ros_node;

  VelocityController velocity_controller(ros_node);

  ros::spin();
}
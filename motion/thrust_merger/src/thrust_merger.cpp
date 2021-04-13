#include "thrust_merger/thrust_merger.h"

ThrustMerger::ThrustMerger(ros::NodeHandle nh) : nh(nh)
{
  // parameters
  std::string dp_topic;
  std::string los_topic;
  std::string vel_topic;
  std::string joy_topic;
  std::string output_topic;

  if (!nh.getParam("/thrust_merger/rate", rate))
    rate = 30;
  if (!nh.getParam("/thrust_merger/dp_topic", dp_topic))
    dp_topic = "/thrust/dp";
  if (!nh.getParam("/thrust_merger/los_topic", los_topic))
    los_topic = "/thrust/los";
  if (!nh.getParam("/thrust_merger/vel_topic", vel_topic))
    vel_topic = "/thrust/vel";
  if (!nh.getParam("/thrust_merger/joy_topic", joy_topic))
    joy_topic = "/thrust/joy";
  if (!nh.getParam("/thrust_merger/output_topic", output_topic))
    output_topic = "/thrust/combined";

  // init class variables
  dp_wrench = Eigen::Vector6d();
  los_wrench = Eigen::Vector6d();
  vel_wrench = Eigen::Vector6d();
  joy_wrench = Eigen::Vector6d();

  // subscribers and publisher
  dp_sub = nh.subscribe(dp_topic, 1, &ThrustMerger::dpCallback, this);
  los_sub = nh.subscribe(los_topic, 1, &ThrustMerger::losCallback, this);
  vel_sub = nh.subscribe(vel_topic, 1, &ThrustMerger::velCallback, this);
  joy_sub = nh.subscribe(joy_topic, 1, &ThrustMerger::joyCallback, this);
  thrust_pub = nh.advertise<geometry_msgs::Wrench>(output_topic, 1);

  ROS_INFO("thrust_merger initiated");
}

void ThrustMerger::spin()
{
  ros::Rate ros_rate(rate);
  while (ros::ok())
  {
    // execute waiting callbacks
    ros::spinOnce();

    // update and publish combined wrench
    combined_wrench = dp_wrench + los_wrench + vel_wrench + joy_wrench;
    geometry_msgs::Wrench wrench_msg;
    tf::wrenchEigenToMsg(combined_wrench, wrench_msg);
    thrust_pub.publish(wrench_msg);

    // reset local wrenches
    dp_wrench.setZero();
    los_wrench.setZero();
    vel_wrench.setZero();
    joy_wrench.setZero();

    ros_rate.sleep();
  }
}

void ThrustMerger::dpCallback(geometry_msgs::Wrench wrench_msgs)
{
  tf::wrenchMsgToEigen(wrench_msgs, dp_wrench);
}

void ThrustMerger::losCallback(geometry_msgs::Wrench wrench_msgs)
{
  tf::wrenchMsgToEigen(wrench_msgs, los_wrench);
}

void ThrustMerger::velCallback(geometry_msgs::Wrench wrench_msgs)
{
  tf::wrenchMsgToEigen(wrench_msgs, vel_wrench);
}

void ThrustMerger::joyCallback(geometry_msgs::Wrench wrench_msgs)
{
  tf::wrenchMsgToEigen(wrench_msgs, joy_wrench);
}

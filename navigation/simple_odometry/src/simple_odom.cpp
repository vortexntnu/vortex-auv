#include "simple_odom.h"

SimpleOdom::SimpleOdom(ros::NodeHandle nh) : nh(nh)
{
  // get params
  std::string imu_topic;
  std::string dvl_topic;
  std::string odom_topic;
  if (!nh.getParam("simple_odom/imu_topic", imu_topic))
    imu_topic = "/auv/imu";
  if (!nh.getParam("simple_odom/dvl_topic", dvl_topic))
    dvl_topic = "/auv/odom";
  if (!nh.getParam("simple_odom/odom_topic", odom_topic))
    odom_topic = "/odometry/filtered";
  if (!nh.getParam("simple_odom/update_rate", update_rate))
    update_rate = 60.0;

  // subscribers and publishers
  imu_sub = nh.subscribe(imu_topic, 1, &SimpleOdom::imuCallback, this);
  dvl_sub = nh.subscribe(dvl_topic, 1, &SimpleOdom::dvlCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);

  // wait for first imu and dvl msg
  ROS_INFO("Waiting for initial IMU and DVL msgs..");
  ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic, nh);
  ros::topic::waitForMessage<nav_msgs::Odometry>(dvl_topic, nh);

  ROS_INFO("SimpleOdom initialized");
}

void SimpleOdom::spin()
{
  ros::Rate rate(update_rate);
  double dt = rate.expectedCycleTime().toSec();
  ROS_DEBUG_STREAM("cycle time: " << dt);

  while (ros::ok())
  {
    // execute waiting callbacks
    ros::spinOnce();

    // integrate x and y postion
    position[0] = position[0] + linear_vel[0] * dt;
    position[1] = position[1] + linear_vel[1] * dt;

    // create odom msg
    nav_msgs::Odometry odometry_msg;  
    odometry_msg.pose.pose.position.x = position[0];
    odometry_msg.pose.pose.position.y = position[1];
    odometry_msg.pose.pose.position.z = position[2];

    odometry_msg.pose.pose.orientation.x = orientation.x();
    odometry_msg.pose.pose.orientation.y = orientation.y();
    odometry_msg.pose.pose.orientation.z = orientation.z();
    odometry_msg.pose.pose.orientation.w = orientation.w();

    odometry_msg.twist.twist.angular.x = angular_vel[0];
    odometry_msg.twist.twist.angular.y = angular_vel[1];
    odometry_msg.twist.twist.angular.z = angular_vel[2];

    odometry_msg.twist.twist.linear.x = linear_vel[0];
    odometry_msg.twist.twist.linear.y = linear_vel[1];
    odometry_msg.twist.twist.linear.z = linear_vel[2];

    // publish odom
    odom_pub.publish(odometry_msg);

    rate.sleep();
  }
}

void SimpleOdom::imuCallback(const sensor_msgs::Imu& imu_msg)
{
  angular_vel[0] = imu_msg.angular_velocity.x;
  angular_vel[1] = imu_msg.angular_velocity.y;
  angular_vel[2] = imu_msg.angular_velocity.z;

  orientation[0] = imu_msg.orientation.w;
  orientation[1] = imu_msg.orientation.x;  
  orientation[2] = imu_msg.orientation.y;
  orientation[3] = imu_msg.orientation.z;
}

void SimpleOdom::dvlCallback(const nav_msgs::Odometry& odom_msg)
{
  linear_vel[0] = odom_msg.twist.twist.linear.x;
  linear_vel[1] = odom_msg.twist.twist.linear.y;
  linear_vel[2] = odom_msg.twist.twist.linear.z;

  position[2] = odom_msg.pose.pose.position.z;
}

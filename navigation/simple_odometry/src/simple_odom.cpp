#include "simple_odometry/simple_odom.h"

SimpleOdom::SimpleOdom(ros::NodeHandle nh) : nh(nh)
{
  // get params
  std::string imu_topic;
  std::string dvl_topic;
  std::string odom_topic;
  std::string mocap_topic;
  std::string imu_link;
  std::string dvl_link;
  if (!nh.getParam("simple_odom/imu_topic", imu_topic))
    imu_topic = "/auv/imu";
  if (!nh.getParam("simple_odom/dvl_topic", dvl_topic))
    dvl_topic = "/auv/odom";  
  if (!nh.getParam("simple_odom/mocap_topic", mocap_topic))
    mocap_topic = "/qualisys/Body_1/pose";
  if (!nh.getParam("simple_odom/odom_topic", odom_topic))
    odom_topic = "/odometry/filtered";
  if (!nh.getParam("simple_odom/imu_link", imu_link))
    imu_link = "imu_0";
  if (!nh.getParam("simple_odom/dvl_link", dvl_link))
    dvl_link = "dvl_link";

  // set up IMU and DVL transforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  double timeout = 10; // seconds to wait for transforms to become available
  ROS_INFO("Waiting for IMU and DVL transforms..");
  imu_transform = tf_buffer.lookupTransform("base_link", imu_link, ros::Time(0), ros::Duration(timeout));
  dvl_transform = tf_buffer.lookupTransform("base_link", dvl_link, ros::Time(0), ros::Duration(timeout));

  // subscribers and publishers
  imu_sub = nh.subscribe(imu_topic, 1, &SimpleOdom::imuCallback, this);
  dvl_sub = nh.subscribe(dvl_topic, 1, &SimpleOdom::dvlCallback, this);
  mocap_sub = nh.subscribe(mocap_topic, 1, &SimpleOdom::mocapCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);

  // wait for first imu, dvl and mocap msg
  ROS_INFO("Waiting for initial IMU, DVL and MOCAP msgs..");
  ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic, nh);
  ros::topic::waitForMessage<geometry_msgs::TwistWithCovarianceStamped>(dvl_topic, nh);
  ros::topic::waitForMessage<geometry_msgs::PoseStamped>(mocap_topic, nh);

  ROS_INFO("SimpleOdom initialized");
}

void SimpleOdom::spin()
{
  ros::Rate rate(update_rate);

  while (ros::ok())
  {
    // execute waiting callbacks
    ros::spinOnce();

    // create odom msg
    nav_msgs::Odometry odometry_msg;  

    odometry_msg.pose.pose.position = tf2::toMsg(position);
    odometry_msg.pose.pose.orientation = tf2::toMsg(orientation);

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
  Eigen::Vector3d angular_vel_imu;
  tf2::fromMsg(imu_msg.angular_velocity, angular_vel_imu);
  tf2::doTransform(angular_vel_imu, angular_vel, imu_transform);
}

void SimpleOdom::dvlCallback(const geometry_msgs::TwistWithCovarianceStamped& twist_msg)
{
  Eigen::Vector3d linear_vel_dvl;
  tf2::fromMsg(twist_msg.twist.twist.linear, linear_vel_dvl);
  tf2::doTransform(linear_vel_dvl, linear_vel, dvl_transform);
}

void SimpleOdom::mocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose.position, position);
  tf2::fromMsg(msg.pose.orientation, orientation);
}

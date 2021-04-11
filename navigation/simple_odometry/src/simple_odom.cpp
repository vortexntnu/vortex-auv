#include "simple_odom.h"

SimpleOdom::SimpleOdom(ros::NodeHandle nh) : nh(nh)
{
    // get params

    // subscribers and publishers

    imu_sub = nh.subscribe("/auv/imu", 1, &SimpleOdom::imuCallback, this);
    dvl_sub = nh.subscribe("/auv/odom", 1, &SimpleOdom::dvlCallback, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);

    // wait for first imu and dvl msg

   ros::topic::waitForMessage<nav_msgs::Odometry>("/auv/odom", nh);
   ros::topic::waitForMessage<sensor_msgs::Imu>("/auv/imu", nh);


    ROS_INFO("SimpleOdom initialized");
}

void SimpleOdom::spin()
{
    while(ros::ok()){

    // create odom msg
    nav_msgs::Odometry odometry_msg; // consider changing variable name
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

    ros::spinOnce();
    ros::Rate rate(50);
    rate.sleep();

    
    }
}

void SimpleOdom::imuCallback(const sensor_msgs::Imu &imu_msg)
{
    angular_vel = Eigen::Vector3d(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    orientation = Eigen::Quaterniond(imu_msg.orientation.w, imu_msg.orientation.x,
                                     imu_msg.orientation.y, imu_msg.orientation.z);
}

void SimpleOdom::dvlCallback(const nav_msgs::Odometry &odom_msg)
{
    linear_vel = Eigen::Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);

    //TODO: calculate position.x, position.y (numerical integration)
    position = Eigen::Vector3d(0, 0, odom_msg.pose.pose.position.z);
}

int main(int argc, char **argv)
{
    const bool DEBUG_MODE = false; // debug logs are printed to console when true

    ros::init(argc, argv, "simple_odometry");
    ros::NodeHandle nh;

    if (DEBUG_MODE)
    {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        ros::console::notifyLoggerLevelsChanged();
    }

    // ros::Rate rate(50);
    SimpleOdom simple_odom(nh);
    simple_odom.spin();
}

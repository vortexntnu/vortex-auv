#ifndef SIMPLE_ODOM_H
#define SIMPLE_ODOM_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// These typdefs are lacking from the default eigen namespace
namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

class SimpleOdom
{
    public:
    void spin();
    SimpleOdom(ros::NodeHandle nh);


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
    void imuCallback(const sensor_msgs::Imu& imu_msg);
    void dvlCallback(const nav_msgs::Odometry& odom_msg);
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    ros::Subscriber imu_sub;
    ros::Subscriber dvl_sub;
    ros::Publisher odom_pub;
    ros::NodeHandle nh;

};

#endif

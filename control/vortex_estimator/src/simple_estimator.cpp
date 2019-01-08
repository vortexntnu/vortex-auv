#include "vortex_estimator/simple_estimator.h"

#include <cmath>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

SimpleEstimator::SimpleEstimator()
{
  //m_imu_sub      = m_nh.subscribe("/sensors/imu/data", 1, &SimpleEstimator::imuCallback, this);
  //m_pressure_sub = m_nh.subscribe("/sensors/pressure", 1, &SimpleEstimator::pressureCallback, this);
  m_imu_sub      = m_nh.subscribe("/manta/imu", 1, &SimpleEstimator::imuCallback, this);
  m_pressure_sub = m_nh.subscribe("/manta/pressure", 1, &SimpleEstimator::pressureCallback, this);
  m_state_pub    = m_nh.advertise<vortex_msgs::RovState>("state_estimate", 1);

  if (!m_nh.getParam("atmosphere/pressure", m_atmospheric_pressure))
    ROS_ERROR("Could not read parameter atmospheric pressure.");

  if (!m_nh.getParam("/water/density", m_water_density))
    ROS_ERROR("Could not read parameter water density.");

  if (!m_nh.getParam("/gravity/acceleration", m_gravitational_acceleration))
    ROS_ERROR("Could not read parameter gravititional acceleration.");

  m_state.pose.orientation.w = 1.0;
  m_state.pose.orientation.x = 0.0;
  m_state.pose.orientation.y = 0.0;
  m_state.pose.orientation.z = 0.0;

  ROS_INFO("Initialized.");
}

void SimpleEstimator::imuCallback(const sensor_msgs::Imu &msg)
{
  // Rotation measured by IMU
  Eigen::Quaterniond quat_imu;
  tf::quaternionMsgToEigen(msg.orientation, quat_imu);
  Eigen::Vector3d euler_imu = quat_imu.toRotationMatrix().eulerAngles(2, 1, 0);

  // Alter IMU measurements to Z down, Y right, X forward
  Eigen::Vector3d euler_ned(-euler_imu(0), euler_imu(2) + c_pi, euler_imu(1) + c_pi);

  // Transform back to quaternion
  Eigen::Matrix3d R_ned;
  R_ned = Eigen::AngleAxisd(euler_ned(0), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(euler_ned(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(euler_ned(2), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quat_ned;

  // Convert to quaternion message and publish
  tf::quaternionEigenToMsg(quat_ned, m_state.pose.orientation);
  m_state.twist.angular.z = -msg.angular_velocity.z;
  m_state_pub.publish(m_state);
}

void SimpleEstimator::pressureCallback(const sensor_msgs::FluidPressure &msg)
{
  const float gauge_pressure = msg.fluid_pressure - m_atmospheric_pressure;
  const float depth_meters = gauge_pressure / (m_water_density * m_gravitational_acceleration);
  m_state.pose.position.z = depth_meters;
  m_state_pub.publish(m_state);
}

/*
 * Copyright [2015] [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <mocap_base/MoCapDriverBase.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

namespace mocap {

Subject::Subject(ros::NodeHandle* nptr, const string& sub_name,
    const std::string& p_frame):
  name         (sub_name),
  status       (LOST),
  nh_ptr       (nptr),
  parent_frame (p_frame){

  pub_filter = nh_ptr->advertise<nav_msgs::Odometry>(name+"/odom", 1);
  pub_raw = nh_ptr->advertise<geometry_msgs::PoseStamped>(name+"/pose", 1);
  pub_vel = nh_ptr->advertise<geometry_msgs::TwistStamped>(name+"/velocity", 1);
  return;
}

// Get and set name of the subject
const string& Subject::getName() {
  boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  return name;
}
void Subject::setName(const string& sub_name) {
  boost::unique_lock<boost::shared_mutex> write_lock(mtx);
  name = sub_name;
}

// Enable or diable the subject
const Subject::Status& Subject::getStatus() {
  boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  return status;
}
void Subject::enable() {
  boost::unique_lock<boost::shared_mutex> write_lock(mtx);
  status = INITIALIZING;
  ROS_INFO("Initializing subject %s", name.c_str());
}
void Subject::disable() {
  boost::unique_lock<boost::shared_mutex> write_lock(mtx);
  kFilter.reset();
  status = LOST;
  ROS_WARN("Lost track of subject %s", name.c_str());
}

// Get the state of the subject
const Quaterniond& Subject::getAttitude() {
  boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  return kFilter.attitude;
}
const Vector3d& Subject::getPosition() {
  boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  return kFilter.position;
}
const Vector3d& Subject::getAngularVel() {
  boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  return kFilter.angular_vel;
}
const Vector3d& Subject::getLinearVel() {
  boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  return kFilter.linear_vel;
}

// Set the noise parameter for the kalman filter
bool Subject::setParameters(
    const Matrix<double, 12, 12>& u_cov,
    const Matrix<double, 6, 6>& m_cov,
    const int& freq) {
  boost::unique_lock<boost::shared_mutex> write_lock(mtx);
  return kFilter.init(u_cov, m_cov, freq);
}

// Process the new measurement
void Subject::processNewMeasurement(
    const double& time,
    const Quaterniond& m_attitude,
    const Vector3d& m_position) {
  //ros::Time tbefore_filter = ros::Time::now();
  
  boost::unique_lock<boost::shared_mutex> write_lock(mtx);

  // Publish raw data from mocap system
  geometry_msgs::PoseStamped pose_raw;
  pose_raw.header.stamp = ros::Time(time);
  pose_raw.header.frame_id = parent_frame;

  tf::quaternionEigenToMsg(m_attitude, pose_raw.pose.orientation);
  tf::pointEigenToMsg(m_position, pose_raw.pose.position);

  
  pub_raw.publish(pose_raw);
  if (!kFilter.isReady()) {
    status = INITIALIZING;
    kFilter.prepareInitialCondition(time, m_attitude, m_position);
    return;
  }

  // Transform matrix from parent to child frame
  Isometry3d tf_parent2child = Isometry3d(Translation3d(m_position) * m_attitude);

  status = TRACKED;
  // Perfrom the kalman filter
   
  kFilter.prediction(time);
  kFilter.update(m_attitude, m_position);
  // Publish the new state
  nav_msgs::Odometry odom_filter;
  odom_filter.header.stamp = ros::Time(time);
  odom_filter.header.frame_id = parent_frame;
  odom_filter.child_frame_id = name + "/base_link";
  tf::quaternionEigenToMsg(kFilter.attitude, odom_filter.pose.pose.orientation);
  tf::pointEigenToMsg(kFilter.position, odom_filter.pose.pose.position);
  // Transform twist to child frame
  tf::vectorEigenToMsg(tf_parent2child.linear() * kFilter.angular_vel, odom_filter.twist.twist.angular);
  tf::vectorEigenToMsg(tf_parent2child.linear() * kFilter.linear_vel, odom_filter.twist.twist.linear);
  // To be compatible with the covariance in ROS, we have to do some shifting
  Map<Matrix<double, 6, 6, RowMajor> > pose_cov(odom_filter.pose.covariance.begin());
  Map<Matrix<double, 6, 6, RowMajor> > vel_cov(odom_filter.twist.covariance.begin());
  pose_cov.topLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(3, 3);
  pose_cov.topRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(3, 0);
  pose_cov.bottomLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(0, 3);
  pose_cov.bottomRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(0, 0);
  vel_cov.topLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(9, 9);
  vel_cov.topRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(9, 6);
  vel_cov.bottomLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(6, 9);
  vel_cov.bottomRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(6, 6);
  // Transform the velocity cov to child frame
  Matrix<double, 6, 6, RowMajor> r_vel = Matrix<double, 6, 6, RowMajor>::Zero();	//Zero initialized velocity cov matrix
  r_vel.block<3, 3>(0, 0) = r_vel.block<3, 3>(3, 3) = tf_parent2child.linear();
  vel_cov = r_vel * vel_cov * r_vel.transpose();

  pub_filter.publish(odom_filter);
  //ROS_INFO("filter time: %f", (ros::Time::now() - tbefore_filter).toSec());

  // Publish velocity in parent frame
  geometry_msgs::TwistStamped vel;
  vel.header.stamp = ros::Time(time);
  vel.header.frame_id = parent_frame;

  tf::vectorEigenToMsg(kFilter.angular_vel, vel.twist.angular);
  tf::vectorEigenToMsg(kFilter.linear_vel, vel.twist.linear);

  pub_vel.publish(vel);

  return;
}
}

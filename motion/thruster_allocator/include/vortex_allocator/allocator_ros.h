#ifndef VORTEX_ALLOCATOR_ALLOCATOR_ROS_H
#define VORTEX_ALLOCATOR_ALLOCATOR_ROS_H

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32MultiArray.h>

#include <eigen_conversions/eigen_msg.h>

#include "vortex_allocator/eigen_typedefs.h"
#include "vortex_allocator/eigen_helper.h"
#include "vortex_allocator/pseudoinverse_allocator.h"


class Allocator
{
public:
  explicit Allocator(ros::NodeHandle nh);
  void callback(const geometry_msgs::Wrench &msg) const;
  void thrusterForcesCb(const std_msgs::Float32MultiArray &thruster_forces_msg);
  
private:
  ros::NodeHandle m_nh;

  // for computing thruster forces from body forces
  ros::Subscriber m_sub;
  ros::Publisher  m_pub;

  // for publishing delivered thrust in body
  ros::Subscriber thruster_forces_sub;
  ros::Publisher delivered_forces_pub;

  Eigen::MatrixXd thrust_configuration;
  int m_num_degrees_of_freedom;
  int m_num_thrusters;
  std::map<std::string, bool> m_active_degrees_of_freedom;
  double m_min_thrust;
  double m_max_thrust;
  const double c_force_range_limit = 100;

  std::unique_ptr<PseudoinverseAllocator> m_pseudoinverse_allocator;

  Eigen::VectorXd rovForcesMsgToEigen(const geometry_msgs::Wrench &msg) const;
};

#endif  // VORTEX_ALLOCATOR_ALLOCATOR_ROS_H

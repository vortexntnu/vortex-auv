#include "vortex_allocator/allocator_ros.h"

Allocator::Allocator(ros::NodeHandle nh) : m_nh(nh) {
  // parameters
  if (!m_nh.getParam("/propulsion/dofs/num", m_num_degrees_of_freedom))
    ROS_FATAL("Failed to read parameter number of dofs.");
  if (!m_nh.getParam("/propulsion/thrusters/num", m_num_thrusters))
    ROS_FATAL("Failed to read parameter number of thrusters.");
  if (!m_nh.getParam("/propulsion/dofs/which", m_active_degrees_of_freedom))
    ROS_FATAL("Failed to read parameter which dofs.");

  // Read thrust config matrix
  if (!getMatrixParam(m_nh, "/propulsion/thrusters/configuration_matrix",
                      &thrust_configuration)) {
    ROS_FATAL("Failed to read parameter thrust config matrix. Killing node...");
    ros::shutdown();
  }

  // calculate pseudo inverse of thrust config matrix
  Eigen::MatrixXd thrust_configuration_pseudoinverse;
  if (!pseudoinverse(thrust_configuration,
                     &thrust_configuration_pseudoinverse)) {
    ROS_FATAL("Failed to compute pseudoinverse of thrust config matrix. "
              "Killing node...");
    ros::shutdown();
  }

  // publishers and subscribers
  m_sub =
      m_nh.subscribe("/thrust/desired_forces/flipped", 1, &Allocator::callback, this);
  m_pub =
      m_nh.advertise<std_msgs::Float32MultiArray>("/thrust/thruster_forces", 1);

  delivered_forces_pub =
      m_nh.advertise<geometry_msgs::Wrench>("tau_delivered", 1);
  thruster_forces_sub = m_nh.subscribe("/thrust/delivered_forces", 1,
                                       &Allocator::thrusterForcesCb, this);

  m_pseudoinverse_allocator.reset(
      new PseudoinverseAllocator(thrust_configuration_pseudoinverse));
  ROS_INFO("Initialized.");
}

void Allocator::callback(const geometry_msgs::Wrench &msg_in) const {
  const Eigen::VectorXd rov_forces = rovForcesMsgToEigen(msg_in);

  Eigen::VectorXd thruster_forces =
      m_pseudoinverse_allocator->compute(rov_forces);

  if (isFucked(thruster_forces)) {
    ROS_ERROR("Thruster forces vector invalid, ignoring.");
    return;
  }

  std_msgs::Float32MultiArray msg_out;
  arrayEigenToMsg(thruster_forces, &msg_out);

  m_pub.publish(msg_out);
}

Eigen::VectorXd
Allocator::rovForcesMsgToEigen(const geometry_msgs::Wrench &msg) const {
  Eigen::VectorXd rov_forces(m_num_degrees_of_freedom);
  unsigned i = 0;
  if (m_active_degrees_of_freedom.at("surge"))
    rov_forces(i++) = msg.force.x;
  if (m_active_degrees_of_freedom.at("sway"))
    rov_forces(i++) = msg.force.y;
  if (m_active_degrees_of_freedom.at("heave"))
    rov_forces(i++) = msg.force.z;
  if (m_active_degrees_of_freedom.at("roll"))
    rov_forces(i++) = msg.torque.x;
  if (m_active_degrees_of_freedom.at("pitch"))
    rov_forces(i++) = msg.torque.y;
  if (m_active_degrees_of_freedom.at("yaw"))
    rov_forces(i++) = msg.torque.z;

  if (i != m_num_degrees_of_freedom) {
    ROS_WARN_STREAM("Invalid length of rov_forces vector. Is "
                    << i << ", should be " << m_num_degrees_of_freedom
                    << ". Returning zero thrust vector.");
    return Eigen::VectorXd::Zero(m_num_degrees_of_freedom);
  }

  return rov_forces;
}

void Allocator::thrusterForcesCb(
    const std_msgs::Float32MultiArray &thruster_forces_msg) {
  // convert msg to eigen
  Eigen::Vector8d thruster_forces;
  for (int i = 0; i < 8; i++) {
    thruster_forces[i] = thruster_forces_msg.data[i];
  }

  // calculate forces in body
  Eigen::Vector6d forces_body = thrust_configuration * thruster_forces;

  // convert back to msg
  geometry_msgs::Wrench tau_delivered;
  tf::wrenchEigenToMsg(forces_body, tau_delivered);

  // publish body forces
  delivered_forces_pub.publish(tau_delivered);
}

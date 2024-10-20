#include "../include/dp_controller/main.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <tuple>

using namespace std::chrono_literals;

SMC_node::SMC_node() : Node("SMC_Controller2") {
  wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/wrench_input", 10);

  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/nucleus/odom", 10, std::bind(&SMC_node::odometry_callback, this, std::placeholders::_1));

  // Initialize control variables
  eta_e_prev_global.setZero(6);
  integral_eta_e_global.setZero(6);
  prev_time = this->now();
}

// Function to format eta and nu from the odometry message
std::tuple<Eigen::VectorXd, Eigen::VectorXd> SMC_node::format_eta(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::VectorXd eta(7);
  eta << msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w;

  Eigen::VectorXd nu(6);
  nu << msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z,
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z;

  return std::make_tuple(eta, nu);
}

// Function to convert quaternion to Euler angles (roll, pitch, yaw)
Eigen::Vector3d SMC_node::quaternionToEuler(double w, double x, double y, double z) {
  Eigen::Vector3d euler;

  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  euler(0) = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    euler(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    euler(1) = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  euler(2) = std::atan2(siny_cosp, cosy_cosp);

  return euler; // Return the Euler angles as a Vector3d (roll, pitch, yaw)
}

// Function to compute the smallest signed angle difference
double SMC_node::ssa(const double a2, const double a1) {
  double difference = std::fmod(a2 - a1 + M_PI, 2.0 * M_PI);
  if (difference < 0)
    difference += 2.0 * M_PI;
  difference -= M_PI;
  return difference;
}

// Function to compute the vector of smallest signed angle differences
Eigen::Vector3d SMC_node::ssa_vector(const Eigen::Vector3d &Desired, const Eigen::Vector3d &Current) {
  Eigen::Vector3d ssa_vector_angle;
  ssa_vector_angle(0) = ssa(Desired(0), Current(0));
  ssa_vector_angle(1) = ssa(Desired(1), Current(1));
  ssa_vector_angle(2) = ssa(Desired(2), Current(2));
  return ssa_vector_angle;
}

// Function to compute the difference between desired and current states
Eigen::VectorXd SMC_node::vector_difference(const Eigen::VectorXd &Current, const Eigen::VectorXd &Desired) {
  Eigen::VectorXd difference(6);

  // Position error
  difference.head<3>() = Desired.head<3>() - Current.head<3>();

  // Orientation error
  difference.tail<3>() = ssa_vector(Desired.tail<3>(), Current.tail<3>());

  return difference;
}

// PID controller function
Eigen::VectorXd SMC_node::tau_PID(double dt, const Eigen::VectorXd &nu, const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d) {
  // Define the gains for the controller
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Ki = Eigen::MatrixXd::Zero(6, 6);

  // Position gains
  Kp.diagonal() << 20, 20, 18, 8, 8, 8;
  Kd.diagonal() << 8, 8, 15, 2.5, 2.5, 2.5;
  Ki.diagonal() << 0.5, 0.5, 0.3, 0.3, 0.3, 0.3;

  // Compute the error
  Eigen::VectorXd eta_e = vector_difference(eta, eta_d);

  // Compute the integral of the error
  integral_eta_e_global += eta_e * dt;

  // Anti-windup mechanism
  double max_integral = 100.0; // Define a maximum value for the integral term
  for (int i = 0; i < integral_eta_e_global.size(); ++i) {
    if (integral_eta_e_global[i] > max_integral) {
      integral_eta_e_global[i] = max_integral;
    } else if (integral_eta_e_global[i] < -max_integral) {
      integral_eta_e_global[i] = -max_integral;
    }
  }

  // Compute the derivative of the error
  Eigen::VectorXd eta_e_dot = (eta_e - eta_e_prev_global) / dt;

  // Update the previous error
  eta_e_prev_global = eta_e;

  // PID control law
  Eigen::VectorXd tau = Kp * eta_e + Kd * eta_e_dot + Ki * integral_eta_e_global;

  // Limit the control input
  double max_tau = 70.0; // Define a maximum value for the control input
  for (int i = 0; i < tau.size(); ++i) {
    if (tau[i] > max_tau) {
      tau[i] = max_tau;
    } else if (tau[i] < -max_tau) {
      tau[i] = -max_tau;
    }
  }

  // Print statements for debugging
  std::cout << "eta_e: " << eta_e.transpose() << std::endl;
  std::cout << "integral_eta_e_global: " << integral_eta_e_global.transpose() << std::endl;
  std::cout << "eta_e_dot: " << eta_e_dot.transpose() << std::endl;
  std::cout << "tau before limiting: " << tau.transpose() << std::endl;

  return tau;
}

// Function to compute control input
Eigen::VectorXd SMC_node::run_u(const nav_msgs::msg::Odometry::SharedPtr msg, const Eigen::VectorXd &d_eta, const Eigen::VectorXd &d_nu, double dt) {
  Eigen::VectorXd eta, nu;
  std::tie(eta, nu) = format_eta(msg);

  Eigen::VectorXd eta_temp(6);
  Eigen::Vector3d euler_angles = quaternionToEuler(eta(6), eta(3), eta(4), eta(5)); // w, x, y, z

  // Set the first three components to the position part of eta
  eta_temp.head<3>() = eta.head<3>();

  // Set the last three components to the Euler angles (roll, pitch, yaw)
  eta_temp.tail<3>() = euler_angles;

  Eigen::VectorXd U = tau_PID(dt, nu, eta_temp, d_eta);

  std::cout << "The input given is: \n"
            << U << std::endl;

  return U;
}

// Function to convert Eigen vector to Wrench message
geometry_msgs::msg::Wrench SMC_node::convert_to_wrench(const Eigen::VectorXd &vec) {
  geometry_msgs::msg::Wrench wrench_msg;

  if (vec.size() != 6) {
    throw std::invalid_argument("Vector size must be 6 to convert to Wrench message.");
  } else {
    wrench_msg.force.x = vec[0];
    wrench_msg.force.y = vec[1];
    wrench_msg.force.z = vec[2];

    wrench_msg.torque.x = vec[3];
    wrench_msg.torque.y = vec[4];
    wrench_msg.torque.z = vec[5];
  }
  return wrench_msg;
}

// Callback function for odometry messages
void SMC_node::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  rclcpp::Time current_time = msg->header.stamp;
  double dt = (current_time - prev_time).seconds();
  if (dt <= 0) {
    dt = 0.01; // Set a default small dt
  }
  prev_time = current_time;

  // Define desired eta and nu
  Eigen::VectorXd d_eta(6);
  Eigen::VectorXd d_nu(6);
  d_eta << 1, 1, 1, 0, 0, 0; // Desired position and orientation
  d_nu << 0, 0, 0, 0, 0, 0;  // Desired velocities

  // Call the run_u function with the current odometry message and desired eta and nu
  Eigen::VectorXd tau = run_u(msg, d_eta, d_nu, dt);

  // Convert the resulting tau to a Wrench message
  geometry_msgs::msg::Wrench wrench_msg = convert_to_wrench(tau);

  RCLCPP_INFO(this->get_logger(), "Publishing Wrench: force(%f, %f, %f), torque(%f, %f, %f)",
              wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z,
              wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z);

  wrench_publisher_->publish(wrench_msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SMC_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

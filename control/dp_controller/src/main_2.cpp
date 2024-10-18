#include "../include/dp_controller/main.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <tuple>

using namespace std::chrono_literals;

// Define a global vector to store the previous nu_e for the next iteration
Eigen::VectorXd nu_e_prev_global(6);
Eigen::VectorXd integral_eta_e_global(6);
Eigen::VectorXd eta_e_prev_global(6);
const double m = 30; //Mass of the vehicle

// Initialize it to zero
void initialize_global_vector() {
  nu_e_prev_global.setZero();
  integral_eta_e_global.setZero();
  eta_e_prev_global.setZero();
}

// Decompose the eta message into a 7x1 vector
std::tuple<Eigen::VectorXd, Eigen::VectorXd> format_eta(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::VectorXd eta(7);
  eta << msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z;

  Eigen::VectorXd nu(6);
  nu << msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z,
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z;

  return std::make_tuple(eta, nu);
};

geometry_msgs::msg::Wrench conver_to_wrench(const Eigen::VectorXd &vec) {
  geometry_msgs::msg::Wrench wrench_msg;

  if (vec.size() != 6) {
    // Handle error for incorrect vector size
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

// Publisher class
SMC_node::SMC_node() : Node("SMC_Controller2") {
  wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/wrench_input", 10);

  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/nucleus/odom", 10, std::bind(&SMC_node::odometry_callback, this, std::placeholders::_1));

  kp_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/controller/kp", 10, std::bind(&SMC_node::kp_callback, this, std::placeholders::_1));
  kd_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/controller/kd", 10, std::bind(&SMC_node::kd_callback, this, std::placeholders::_1));
  ki_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/controller/ki", 10, std::bind(&SMC_node::ki_callback, this, std::placeholders::_1));
  ref_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/controller/ref", 10, std::bind(&SMC_node::ref_callback, this, std::placeholders::_1));

};

// --------------------------------------------------------------
// SSA (Smallest signed angle) function
// --------------------------------------------------------------
double ssa(const double a2, const double a1) {
    double difference = fmod(a2 - a1, 2.0 * M_PI);

    // Ensure the difference is within the range [-π, π]
    if (difference > M_PI)
        difference -= 2.0 * M_PI;
    else if (difference < -M_PI)
        difference += 2.0 * M_PI;

    return difference;
}

Eigen::Vector3d ssa_vector(const Eigen::Vector3d &vector1, const Eigen::Vector3d &vector2) {
    double e1 = ssa(vector1(0), vector2(0));
    double e2 = ssa(vector1(1), vector2(1));
    double e3 = ssa(vector1(2), vector2(2));

    Eigen::Vector3d ssa_vector_angle;
    ssa_vector_angle << e1, e2, e3;

    return ssa_vector_angle;
}

Eigen::VectorXd vector_difference(const Eigen::VectorXd &Current, const Eigen::VectorXd &Desired)
{
  Eigen::VectorXd difference(6);

  difference.head<3>() = Desired.head<3>() - Current.head<3>();

  Eigen::Vector3d v1 = Current.tail<3>();
  Eigen::Vector3d v2 = Desired.tail<3>();

  difference.tail<3>() = ssa_vector(v1, v2);

  return difference;
}


// --------------------------------------------------------------


// Function to convert quaternion to Euler angles (roll, pitch, yaw)
Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z) {
    Eigen::Vector3d euler;
    
    // Roll (x-axis rotation)
    euler(0) = std::atan2(2.0 * ((w * x) + (y * z)), 1.0 - 2.0 * ((x * x)+ (y * y)));
    
    // Pitch (y-axis rotation)
    euler(1) = -(3.1415/2) + (2*std::atan2(sqrt(1 + 2*((w*y) - (x*z))), sqrt(1-2*((w*y) + (x*z))));
    
    // Yaw (z-axis rotation)
    euler(2) = std::atan2(2.0 * ((w * z) + (x * y)), 1.0 - 2.0 * (y * y + z * z));
    
    return euler;  // Return the Euler angles as a Vector3d (roll, pitch, yaw)
}

// Function for controller
Eigen::VectorXd tau_PID(const double dt, const Eigen::VectorXd &nu ,const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d) {
  // Defined the tuning variables
  Eigen::MatrixXd Kp(6, 6);
  Eigen::MatrixXd Kd(6, 6);
  Eigen::MatrixXd Ki(6, 6);

  // Define the gains for the controller
  Kp << 16,   0,     0,    0,    0,    0,
       0,    16,     0,    0,    0,    0,
       0,     0,    16,    0,    0,    0,
       0,     0,     0,    7,    0,    0,
       0,     0,     0,    0,    7,    0,
       0,     0,     0,    0,    0,    7;

  Kd <<   5.5,    0,      0,   0,    0,    0,
          0,    5.5,      0,   0,    0,    0,
          0,      0,    5.5,   0,    0,    0,
          0,      0,      0,   0,    0,    0,
          0,      0,      0,   0,    0,    0,
          0,      0,      0,   0,    0,    0;

  Ki << 1.7,    0,    0,     0,    0,      0,
        0,    1.7,    0,     0,    0,      0,
        0,      0,  1.7,     0,    0,      0,
        0,      0,    0,   0.7,    0,      0,
        0,      0,    0,     0,    0.7,    0,
        0,      0,    0,     0,    0,    0.7;

  // Compute the error
  Eigen::VectorXd eta_e = vector_difference(eta, eta_d);

  // Compute the integral of the error
  Eigen::VectorXd integral_eta_e = (integral_eta_e_global + eta_e) * dt;

  // Compute the derivative of the error
  Eigen::VectorXd eta_e_dot = vector_difference(eta_e_prev_global, eta_e) / dt;

  // Update the global variables
  integral_eta_e_global = (integral_eta_e_global + eta_e);
  eta_e_prev_global = eta_e;

  // Anti-windup mechanism
  double max_integral = 100.0; // Define a maximum value for the integral term
  for (int i = 0; i < integral_eta_e.size(); ++i) {
    if (integral_eta_e[i] > max_integral) {
      integral_eta_e[i] = max_integral;
    } 
    else if (integral_eta_e[i] < -max_integral) {
    integral_eta_e[i] = -max_integral;
    }
  }

  // Use the kp, kd, and ki matrices from the class
  Eigen::VectorXd tau = Kp * eta_e + Kd * eta_e_dot + Ki * integral_eta_e;

  // limit the control input
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
  std::cout << "integral_eta_e: " << integral_eta_e.transpose() << std::endl;
  std::cout << "eta_e_dot: " << eta_e_dot.transpose() << std::endl;
  std::cout << "tau before limiting: " << tau.transpose() << std::endl;

  return tau;
}

Eigen::VectorXd run_u(const nav_msgs::msg::Odometry::SharedPtr msg, Eigen::VectorXd d_eta, Eigen::VectorXd d_nu) {

  Eigen::VectorXd eta, nu;
  std::tie(eta, nu) = format_eta(msg);

  Eigen::VectorXd eta_temp(6);
  Eigen::VectorXd euler_angles = quaternionToEuler(eta(6), eta(3), eta(4), eta(5));
  // Set the first three components to the position part of eta
  eta_temp.head<3>() = eta.head<3>();

  // Set the last three components to the Euler angles (roll, pitch, yaw)
  eta_temp.tail<3>() = euler_angles;

  Eigen::VectorXd U = tau_PID(0.001, Mrb + Ma, nu - d_nu, eta_temp, d_eta);

  std::cout << "The input given is: \n"
            << U << std::endl;

  return U;
}

Eigen::MatrixXd createDiagonalMatrix(const std_msgs::msg::Float64MultiArray::SharedPtr& data)
{
    // Create a 6x6 zero matrix
    Eigen::Matrix<double, 6, 6> matrix = Eigen::Matrix<double, 6, 6>::Zero();

    // Assign the elements to the diagonal
    for (int i = 0; i < 6; ++i)
    {
        matrix(i, i) = data->data[i];
    }

    return matrix;
}

// Callback function for the odometry message
void SMC_node::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Define desired eta and nu (these should be set according to your control requirements)
  Eigen::VectorXd d_eta(6);
  Eigen::VectorXd d_nu(6);
  d_eta << 0, 0, 0, 1, 0, 0;
  d_nu << 0, 0, 0, 0, 0, 0;

  RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f, %f, %f, %f, %f, %f]", msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z,
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z,
              msg->pose.pose.orientation.w);

  // Call the run_u function with the current odometry message and desired eta and nu
  Eigen::VectorXd tau = run_u(msg, d_eta, d_nu);

  // Convert the resulting tau to a Wrench message
  geometry_msgs::msg::Wrench wrench_msg = conver_to_wrench(tau);

  RCLCPP_INFO(this->get_logger(), "Publishing Wrench: force(%f, %f, %f), torque(%f, %f, %f)",
              wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z,
              wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z);

  wrench_publisher_->publish(wrench_msg);
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  initialize_global_vector();
  auto node = std::make_shared<SMC_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
};

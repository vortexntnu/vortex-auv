#ifndef MAIN_HPP
#define MAIN_HPP

// Include necessary headers
#include "Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>

class SMC_node : public rclcpp::Node {
public:
  SMC_node();

private:
  void Wrench_callback();
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  // Helper functions
  std::tuple<Eigen::VectorXd, Eigen::VectorXd> format_eta(const nav_msgs::msg::Odometry::SharedPtr msg);
  Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z);
  double ssa(const double a2, const double a1);
  Eigen::Vector3d ssa_vector(const Eigen::Vector3d &Desired, const Eigen::Vector3d &Current);
  Eigen::VectorXd vector_difference(const Eigen::VectorXd &Current, const Eigen::VectorXd &Desired);
  Eigen::VectorXd tau_PID(double dt, const Eigen::VectorXd &nu, const Eigen::VectorXd &eta, const Eigen::VectorXd &eta_d);
  Eigen::VectorXd run_u(const nav_msgs::msg::Odometry::SharedPtr msg, const Eigen::VectorXd &d_eta, const Eigen::VectorXd &d_nu, double dt);
  geometry_msgs::msg::Wrench convert_to_wrench(const Eigen::VectorXd &vec);

  // Control variables
  Eigen::VectorXd eta_e_prev_global;
  Eigen::VectorXd integral_eta_e_global;
  rclcpp::Time prev_time;

};

#endif

#ifndef MAIN_HPP
#define MAIN_HPP

// Include necessary headers
#include "Eigen/Dense"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

class SMC_node : public rclcpp::Node {
public:
  SMC_node();

private:
  void Wrench_callback();
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_publisher_;

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

#endif

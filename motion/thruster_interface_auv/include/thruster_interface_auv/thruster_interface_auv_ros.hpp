#ifndef THRUSTER_INTERFACE_AUV_NODE_HPP
#define THRUSTER_INTERFACE_AUV_NODE_HPP

#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

class ThrusterInterfaceAUVNode : public rclcpp::Node {
public:
  ThrusterInterfaceAUVNode();

private:
  void thruster_forces_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg);
  void timer_callback();

  rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr thruster_forces_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr thruster_pwm_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  ThrusterInterfaceAUVDriver thruster_driver_;

  std::vector<double> thruster_forces_array_;
  double thrust_timer_period_;
};

#endif // THRUSTER_INTERFACE_AUV_NODE_HPP

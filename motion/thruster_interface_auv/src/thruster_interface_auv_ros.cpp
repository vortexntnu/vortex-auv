#include "thruster_interface_auv/thruster_interface_auv_ros.hpp"

ThrusterInterfaceAUVNode::ThrusterInterfaceAUVNode() : Node("thruster_interface_auv_node") {

  // create a subscriber that takes data from thruster forces and publisher for debugging
  this->thruster_forces_subscriber_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
      "thrust/thruster_forces", 10,
      std::bind(&ThrusterInterfaceAUVNode::thruster_forces_callback, this, std::placeholders::_1));
  this->thruster_pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("pwm", 10);

  // from orca.yaml parameters
  this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_to_pin_mapping");
  this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_direction");
  this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_min");
  this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_max");
  this->declare_parameter<double>("propulsion.thrusters.thrust_update_rate");
  // from thruster_interface_auv.yaml parameters
  this->declare_parameter<int>("i2c.bus");
  this->declare_parameter<int>("i2c.address");
  this->declare_parameter<std::vector<double>>("coeffs.16V.LEFT");
  this->declare_parameter<std::vector<double>>("coeffs.16V.RIGHT");

  int i2c_bus = this->get_parameter("i2c.bus").as_int();
  int i2c_address = this->get_parameter("i2c.address").as_int();
  auto thruster_mapping = this->get_parameter("propulsion.thrusters.thruster_to_pin_mapping").as_integer_array();
  auto thruster_direction = this->get_parameter("propulsion.thrusters.thruster_direction").as_integer_array();
  auto thruster_PWM_min = this->get_parameter("propulsion.thrusters.thruster_PWM_min").as_integer_array();
  auto thruster_PWM_max = this->get_parameter("propulsion.thrusters.thruster_PWM_max").as_integer_array();
  this->thrust_timer_period_ = 1.0 / this->get_parameter("propulsion.thrusters.thrust_update_rate").as_double();
  auto left_coeffs = this->get_parameter("coeffs.16V.LEFT").as_double_array();
  auto right_coeffs = this->get_parameter("coeffs.16V.RIGHT").as_double_array();

  // Initialize thruster driver
  this->thruster_driver_ = ThrusterInterfaceAUVDriver(
      i2c_bus,
      i2c_address,
      std::vector<int>(thruster_mapping.begin(), thruster_mapping.end()),
      std::vector<int>(thruster_direction.begin(), thruster_direction.end()),
      std::vector<int>(thruster_PWM_min.begin(), thruster_PWM_min.end()),
      std::vector<int>(thruster_PWM_max.begin(), thruster_PWM_max.end()),
      std::vector<double>(left_coeffs.begin(), left_coeffs.end()),
      std::vector<double>(right_coeffs.begin(), right_coeffs.end()));

  // Declare "thruster_forces_array" in case no topic comes in at the beginning
  this->thruster_forces_array_ = std::vector<double>(8, 0.00);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(this->thrust_timer_period_),
      std::bind(&ThrusterInterfaceAUVNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "\"thruster_interface_auv_node\" has been started");
}

void ThrusterInterfaceAUVNode::thruster_forces_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
  this->thruster_forces_array_ = msg->thrust;
}

void ThrusterInterfaceAUVNode::timer_callback() {
  std::vector<int16_t> thruster_pwm_array = thruster_driver_.drive_thrusters(this->thruster_forces_array_);

  // publish PWM values for debugging
  std_msgs::msg::Int16MultiArray pwm_message;
  pwm_message.data = thruster_pwm_array;
  thruster_pwm_publisher_->publish(pwm_message);
}

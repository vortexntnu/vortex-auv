#include "thruster_allocator_auv/thruster_allocator_ros.hpp"
#include "thruster_allocator_auv/pseudoinverse_allocator.hpp"
#include "thruster_allocator_auv/thruster_allocator_utils.hpp"
#include <vortex_msgs/msg/thruster_forces.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ThrusterAllocator::ThrusterAllocator()
    : Node("thruster_allocator_node"),
      pseudoinverse_allocator_(Eigen::MatrixXd::Zero(6, 8)) {
  declare_parameter("propulsion.dofs.num", 6);
  declare_parameter("propulsion.thrusters.num", 8);
  declare_parameter("propulsion.thrusters.min", -100);
  declare_parameter("propulsion.thrusters.max", 100);
  declare_parameter("propulsion.thrusters.configuration_matrix",
                    std::vector<double>{0});

  num_dof_ = get_parameter("propulsion.dofs.num").as_int();
  num_thrusters_ = get_parameter("propulsion.thrusters.num").as_int();
  min_thrust_ = get_parameter("propulsion.thrusters.min").as_int();
  max_thrust_ = get_parameter("propulsion.thrusters.max").as_int();
  thrust_configuration = double_array_to_eigen_matrix(
      get_parameter("propulsion.thrusters.configuration_matrix")
          .as_double_array(),
      num_dof_, num_thrusters_);

  subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "thrust/wrench_input", 1,
      std::bind(&ThrusterAllocator::wrench_cb, this, std::placeholders::_1));

  publisher_ = this->create_publisher<vortex_msgs::msg::ThrusterForces>(
      "thrust/thruster_forces", 1);

  timer_ = this->create_wall_timer(
      100ms, std::bind(&ThrusterAllocator::calculate_thrust_timer_cb, this));

  pseudoinverse_allocator_.T_pinv =
      calculate_right_pseudoinverse(thrust_configuration);

  body_frame_forces_.setZero();
}

void ThrusterAllocator::calculate_thrust_timer_cb() {
  Eigen::VectorXd thruster_forces =
      pseudoinverse_allocator_.calculate_allocated_thrust(body_frame_forces_);

  if (is_invalid_matrix(thruster_forces)) {
    RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid");
    return;
  }

  if (!saturate_vector_values(thruster_forces, min_thrust_, max_thrust_)) {
    RCLCPP_WARN(get_logger(), "Thruster forces vector required saturation.");
  }

  vortex_msgs::msg::ThrusterForces msg_out;
  array_eigen_to_msg(thruster_forces, msg_out);
  publisher_->publish(msg_out);
}

void ThrusterAllocator::wrench_cb(const geometry_msgs::msg::Wrench &msg) {
  Eigen::Vector6d msg_vector;
  msg_vector(0) = msg.force.x;  // surge
  msg_vector(1) = msg.force.y;  // sway
  msg_vector(2) = msg.force.z;  // heave
  msg_vector(3) = msg.torque.x; // roll
  msg_vector(4) = msg.torque.y; // pitch
  msg_vector(5) = msg.torque.z; // yaw

  if (!healthy_wrench(msg_vector)) {
    RCLCPP_ERROR(get_logger(), "ASV wrench vector invalid, ignoring.");
    return;
  }
  std::swap(msg_vector, body_frame_forces_);
}

bool ThrusterAllocator::healthy_wrench(const Eigen::VectorXd &v) const {
  if (is_invalid_matrix(v))
    return false;

  bool within_max_thrust = std::none_of(v.begin(), v.end(), [this](double val) {
    return std::abs(val) > max_thrust_;
  });

  return within_max_thrust;
}

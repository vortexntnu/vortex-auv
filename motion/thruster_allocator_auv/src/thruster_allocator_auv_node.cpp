#include "thruster_allocator_auv/thruster_allocator_ros.hpp"
#include "thruster_allocator_auv/thruster_allocator_utils.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto allocator = std::make_shared<ThrusterAllocator>();
  RCLCPP_INFO(allocator->get_logger(), "Thruster allocator initiated");
  rclcpp::spin(allocator);
  rclcpp::shutdown();
  return 0;
}
#include "thruster_interface_auv_cpp/thruster_interface_auv_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started thruster_interface_auv_cpp_node");
    auto node = std::make_shared<ThrusterInterfaceAUVNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
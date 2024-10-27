#include "thruster_interface_auv/thruster_interface_auv_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Started thruster_interface_auv_node");
    rclcpp::spin(std::make_shared<ThrusterInterfaceAUVNode>());
    rclcpp::shutdown();
    return 0;
}

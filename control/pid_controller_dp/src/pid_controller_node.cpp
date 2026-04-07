#include "pid_controller_dp/pid_controller_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

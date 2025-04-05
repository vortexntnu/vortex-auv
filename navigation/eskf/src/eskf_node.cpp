#include "eskf/eskf_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    spdlog::info("Starting ESKF Node");
    rclcpp::spin(std::make_shared<ESKFNode>());
    rclcpp::shutdown();
    return 0;
}

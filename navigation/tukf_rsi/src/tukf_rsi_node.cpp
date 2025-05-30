#include "tukf_rsi/tukf_rsi_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    spdlog::info("Starting TUFK for RSI ROS2 Node");
    rclcpp::spin(std::make_shared<TUKFNode>());
    rclcpp::shutdown();
    return 0;
}
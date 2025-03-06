#include <thread>
#include "los_guidance/los_guidance_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started LOS Guidance node");

    auto node = std::make_shared<LOSGuidanceNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    return 0;
}

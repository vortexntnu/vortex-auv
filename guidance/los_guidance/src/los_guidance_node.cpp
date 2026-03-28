#include <thread>

#include "los_guidance/los_guidance_ros.hpp"

// Main Entry Point
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<vortex::guidance::los::LosGuidanceNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    return 0;
}

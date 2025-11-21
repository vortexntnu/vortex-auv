#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include "waypoint_manager/waypoint_manager.hpp"

int main(int argc, char** argv) {
    auto console = spdlog::stdout_color_mt("waypoint_manager");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
    spdlog::info("Starting Waypoint Manager");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    spdlog::info("Starting executor - Waypoint Manager is running");
    executor.spin();
    spdlog::info("Shutting down Waypoint Manager");
    rclcpp::shutdown();
    return 0;
}

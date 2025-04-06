#include <rclcpp/rclcpp.hpp>
#include "waypoint_manager/waypoint_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("waypoint_manager"),
                "Starting Waypoint Manager node");
    auto node = std::make_shared<WaypointManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

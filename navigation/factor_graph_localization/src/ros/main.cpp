#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <exception>
#include <memory>

#include "factor_graph_localization/ros/navigation_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int result = EXIT_SUCCESS;
    try {
        rclcpp::spin(std::make_shared<vortex::navigation::NavigationNode>());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("factor_graph_localization"), "%s",
                     error.what());
        result = EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return result;
}

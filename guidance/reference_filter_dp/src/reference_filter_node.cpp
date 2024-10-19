#include <reference_filter_dp/reference_filter_ros.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started reference filter node");
    rclcpp::spin(std::make_shared<ReferenceFilterNode>());
    rclcpp::shutdown();
    return 0;
}
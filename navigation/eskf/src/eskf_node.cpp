#include "eskf/eskf_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started ESKF Node");
    rclcpp::spin(std::make_shared<ESKFNode>());
    rclcpp::shutdown();
    return 0;
}

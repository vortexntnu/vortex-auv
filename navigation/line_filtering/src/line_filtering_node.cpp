#include "line_filtering/line_filtering_ros.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera3DPointsNode>());
    rclcpp::shutdown();
    return 0;
}
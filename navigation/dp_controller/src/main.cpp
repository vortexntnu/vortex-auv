#include "rclcpp/rclcpp.hpp"
#include "main.hpp"




int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DPController>());
    rclcpp::shutdown();
    return 0;
}


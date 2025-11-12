#include <spdlog/spdlog.h>
#include "pid_controller_dp/pid_controller_ros.hpp"

auto start_msg = R"(
  ____ ___ ____     ____            _             _ _           
 |  _ \_ _|  _ \   / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __ 
 | |_) | || | | | | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
 |  __/| || |_| | | |__| (_) | | | | |_| | | (_) | | |  __/ |   
 |_|  |___|____/   \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|   
)";

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started PID Controller Node");
    spdlog::info(start_msg);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}

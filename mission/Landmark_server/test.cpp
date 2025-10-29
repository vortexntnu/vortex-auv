// /home/philliab/ros2_ws/src/vortex-auv/mission/Landmark_server/test.cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LandmarkServer : public rclcpp::Node
{
public:
    LandmarkServer()
    : Node("landmark_server")
    {
        RCLCPP_INFO(this->get_logger(), "landmark_server node started");

        pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        timer_ = this->create_wall_timer(500ms, [this]() {
            auto msg = std_msgs::msg::String();
            msg.data = "hello from landmark_server";
            pub_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "published: '%s'", msg.data.c_str());
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkServer>());
    rclcpp::shutdown();
    return 0;
}
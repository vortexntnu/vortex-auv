#include "rclcpp/rclcpp.hpp"

class LandmarkServer : public rclcpp::Node
{
public:
    LandmarkServer() : Node("landmark_server")
    {
        RCLCPP_INFO(this->get_logger(), "Landmark server node started!");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkServer>());
    rclcpp::shutdown();
    return 0;
}


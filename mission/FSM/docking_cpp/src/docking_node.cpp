// This code should publish the docking state to the topic /docking_state
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::chrono_literals;

class DockingNode : public rclcpp::Node {
public:
  DockingNode() : Node("docking_node") {
    docking_state_pub_ = this->create_publisher<std_msgs::msg::String>("/docking_state", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DockingNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();

    message.data = "";
    docking_state_pub_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docking_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingNode>());
  rclcpp::shutdown();
  return 0;
}

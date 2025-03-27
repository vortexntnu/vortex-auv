#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "vortex_utils/cpp_utils.hpp"

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleServiceClientNode : public rclcpp::Node {
   public:
    explicit LifecycleServiceClientNode(
        const std::string& managed_lifecycle_node_name);

    bool change_state(std::uint8_t transition,
                      std::chrono::seconds time_out = std::chrono::seconds(2));
    std::uint8_t get_state(
        const std::chrono::seconds time_out = std::chrono::seconds(2));

   protected:
    rclcpp::Node::SharedPtr node_;
    std::string managed_lifecycle_node_name_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
        change_state_;
};

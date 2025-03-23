#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleManagerClient : public rclcpp::Node {
   public:
    LifecycleManagerClient(const std::string& lifecycle_node_name)
        : Node("LifecycleManagerClient") {}

   private:
};

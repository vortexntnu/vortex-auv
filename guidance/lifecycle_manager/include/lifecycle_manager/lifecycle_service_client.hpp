#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait) {
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;

    do {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)) {
            break;
        }
        status = future.wait_for((time_left < wait_period) ? time_left
                                                           : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);

    return status;
}

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleServiceClient : public rclcpp::Node {
   public:
    LifecycleServiceClient(std::string lifecycle_node_name) {
        get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
            lifecycle_node_name + "/get_state");

        change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            lifecycle_node_name + "/change_state");

        lifecycle_node_name_ = lifecycle_node_name;
    }

    bool change_state(std::uint8_t transition);
    std::uint8_t get_state(
        const std::chrono::seconds timeout = std::chrono::seconds(2));

   protected:
    rclcpp::Node::SharedPtr node_;
    std::string lifecycle_node_name_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
        change_state_;
};

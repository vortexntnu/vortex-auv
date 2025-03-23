#include "lifecycle_service_client.hpp"

LifecycleServiceClient::LifecycleServiceClient(
    const std::string& lifecycle_node_name)
    : rclcpp::Node("") {}

uint8_t LifecycleServiceClient::get_state(std::chrono::seconds time_out) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!get_state_->wait_for_service(time_out)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s is not available.",
                     get_state_->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = get_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(),
                     "Server time out while getting current state for node %s",
                     lifecycle_node_name_);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    // We have an successful answer. So let's print the current state.
    if (future_result.get()) {
        RCLCPP_INFO(this->get_logger(), "Node %s has current state %s.",
                    lifecycle_node_name_,
                    future_result.get()->current_state.label.c_str());
        return future_result.get()->current_state.id;
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s",
                     lifecycle_node_name_);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

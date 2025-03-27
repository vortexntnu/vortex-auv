#include "lifecycle_service_client.hpp"

LifecycleServiceClientNode::LifecycleServiceClientNode(
    const std::string& managed_lifecycle_node_name)
    : Node(managed_lifecycle_node_name + "/client") {
    get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
        managed_lifecycle_node_name + "/get_state");

    change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        managed_lifecycle_node_name + "/change_state");

    managed_lifecycle_node_name_ = managed_lifecycle_node_name;
}

uint8_t LifecycleServiceClientNode::get_state(std::chrono::seconds time_out) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!get_state_->wait_for_service(time_out)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s is not available.",
                     get_state_->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // send request for current state and use the template created to wait for
    // response
    auto future_result = get_state_->async_send_request(request);
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(),
                     "Timed out when attempting to get state from: %s",
                     managed_lifecycle_node_name_);

        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an successful answer. So let's print the current state.
    if (future_result.get()) {
        RCLCPP_INFO(this->get_logger(), "Node %s has current state %s.",
                    managed_lifecycle_node_name_,
                    future_result.get()->current_state.label.c_str());

        return future_result.get()->current_state.id;

    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to get current state for node %s",
                     managed_lifecycle_node_name_);

        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

bool LifecycleServiceClientNode::change_state(std::uint8_t transition,
                                              std::chrono::seconds time_out) {
    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!change_state_->wait_for_service(time_out)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s is not available.",
                     change_state_->get_service_name());

        return false;
    }

    // We send the request with the transition we want to invoke, and use
    // template to wait for response
    auto future_result = change_state_->async_send_request(request);
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(),
                     "Server time out while getting current state for node %s",
                     managed_lifecycle_node_name_);

        return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
        RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.",
                    static_cast<int>(transition));

        return true;

    } else {
        RCLCPP_WARN(get_logger(), "Failed to trigger transition %u",
                    static_cast<unsigned int>(transition));

        return false;
    }
}

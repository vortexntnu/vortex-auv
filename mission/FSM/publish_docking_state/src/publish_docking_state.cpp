#include "publish_docking_state/publish_docking_state.hpp"

PublishDockingState::PublishDockingState()
    : Node("publish_docking_state_node"),
      none_state_(-1),
      last_state_id_(none_state_) {
    this->declare_parameter("topics.fsm.active_controller", "");

    std::string publish_topic =
        this->get_parameter("topics.fsm.active_controller").as_string();

    RCLCPP_DEBUG(this->get_logger(), "Publishing to topic: %s",
                 publish_topic.c_str());

    publisher_ =
        this->create_publisher<std_msgs::msg::String>(publish_topic, 2);
    subscription_ = this->create_subscription<yasmin_msgs::msg::StateMachine>(
        "/fsm_viewer", 10,
        std::bind(&PublishDockingState::listener_callback, this,
                  std::placeholders::_1));
}

void PublishDockingState::listener_callback(
    const yasmin_msgs::msg::StateMachine::SharedPtr fsm_msg) {
    int32_t state_id = fsm_msg->states[0].current_state;

    if (last_state_id_ == state_id || state_id == none_state_) {
        return;
    }

    std::string current_state_name = fsm_msg->states[state_id].name;
    std::string controller_message = get_controller_message(current_state_name);

    if (controller_message != "None") {
        auto msg = std_msgs::msg::String();
        msg.data = controller_message;

        RCLCPP_DEBUG(this->get_logger(), "Message to publish: %s",
                     msg.data.c_str());

        publisher_->publish(msg);
        last_state_id_ = state_id;
    }
}

std::string PublishDockingState::get_controller_message(
    const std::string& current_state){
    {if (current_state == "APPROACH_DOCKING_STATION"){return "LQR";
}
else if (current_state == "GO_ABOVE_DOCKING_STATION" ||
         current_state == "CONVERGE_DOCKING_STATION" ||
         current_state == "RETURN_HOME") {
    return "PID";
}
else {
    return "None";
}
}
}
;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishDockingState>());
    rclcpp::shutdown();
    return 0;
}

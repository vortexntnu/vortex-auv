#ifndef PUBLISH_DOCKING_STATE_HPP
#define PUBLISH_DOCKING_STATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"

class PublishDockingState : public rclcpp::Node {
   private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<yasmin_msgs::msg::StateMachine>::SharedPtr
        subscription_;
    int32_t none_state_;
    int32_t last_state_id_;

   public:
    PublishDockingState();
    void listener_callback(
        const yasmin_msgs::msg::StateMachine::SharedPtr fsm_msg);
    std::string get_controller_message(const std::string& current_state);
};

#endif

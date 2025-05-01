#include "include/joystick_interface_auv/joy_processor_ros.hpp"

JoystickInterfaceNode::JoystickInterfaceNode()
    : Node("joystick_interface_node"), joystick_interface_(this->get_node()) {
    get_parameters();
    set_publishers_and_subscribers();
}

void JoystickInterfaceNode::get_parameters() {
    std::vector<std::string> gain_params {
        "joystick_surge_gain", "joystick_sway_gain", "joystick_yaw_gain",
            "joystick_heave_gain", "joystick_pitch_gain", "joystick_roll_gain",
            "guidance_surge_gain", "guidance_sway_gain", "guidance_yaw_gain",
            "guidance_heave_gain", "guidance_pitch_gain", "guidance_roll_gain",
            "debounce_duration",
    }

    for (const auto& param : gain_params) {
        this->declare_parameter("gains." + param, 1.0);
    };

    std::vector<std::string> topic_params{
        "pose", "joy", "wrench_input", "killswitch", "operation_mode",
    };

    for (const auto& param : topic_params) {
        this->declare_parameter("topics." + param, "_");
    };

    this->declare_parameter("topics.guidance.dp", "_");
    guidance_topic_ = this->get_parameter("topics.guidance.dp").as_string();
}

void JoystickInterfaceNode::set_publishers_and_subscribers() {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, 10,
        std::bind(&JoystickInterfaceNode::joystick_cb, this,
                  std::placeholders::_1));
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, 10,
        std::bind(&JoystickInterfaceNode::pose_cb, this,
                  std::placeholders::_1));
}

void JoystickInterfaceNode::pose_cb(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_state = pose_from_ros(msg.pose.pose);
}

void JoystickInterfaceNode::joystick_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg) {}

void JoystickInterfaceNode::transition_to_xbox_mode() {
    operational_mode_signal_publisher.publish("XBOX");
    mode = JoyStates::XBOX;
}

void JoystickInterfaceNode::transition_to_reference_mode() {}

void JoystickInterfaceNode::transition_to_autonomous_mode() {}

void JoystickInterfaceNode::handle_killswitch_button() {
    joystick_interface_.handle_killswitch();
}

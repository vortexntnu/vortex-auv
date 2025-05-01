#include "include/joystick_interface_auv/joy_processor_auv.hpp"

joystick_interface_auv::JoyProcessor::JoyProcessor(State joy_gains,
                                                   State reference_gains)
    : joy_gains_(joy_gains),
      reference_gains_(reference_gains),
      drone_state_(State()),
      reference_state_(State()),
      _mode(""),
      _last_button_press_time(0.0),
      _last_msg_time(0.0) {
    joystick_buttons_map_ = populate_buttons_map(sensor_msgs::msg::Joy());
    joystick_axes_map_ = populate_axes_map(sensor_msgs::msg::Joy());
}

joystick_interface_auv::JoyProcessor::~JoyProcessor() {}

vortex_msgs::msg::ReferenceFilter
joystick_interface_auv::JoyProcessor::create_reference_message() {
    vortex_msgs::msg::ReferenceFilter reference_msg;

    return reference_msg;
}

vortex_msgs::msg::Wrench
joystick_interface_auv::JoyProcessor::create_wrench_message() {
    vortex_msgs::msg::Wrench wrench_msg;

    return wrench_msg;
}

void joystick_interface_auv::JoyProcessor::check_number_of_buttons(
    sensor_msgs::msg::Joy msg) {}

std::unordered_map<std::string, int>
joystick_interface_auv::JoyProcessor::populate_buttons_map(
    sensor_msgs::msg::Joy msg) {
    std::unordered_map<std::string, int> buttons_map;

    return buttons_map;
}

std::unordered_map<std::string, int>
joystick_interface_auv::JoyProcessor::populate_axes_map(
    sensor_msgs::msg::Joy msg) {
    std::unordered_map<std::string, int> axes_map;

    return axes_map;
}

void joystick_interface_auv::JoyProcessor::calculate_movement(
    std::unordered_map<std::string, int> button_map,
    std::unordered_map<std::string, int> axis_map) {}

void joystick_interface_auv::JoyProcessor::update_reference(double dt) {}

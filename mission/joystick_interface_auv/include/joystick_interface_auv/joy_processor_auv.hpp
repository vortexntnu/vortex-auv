#ifndef JOY_PROCESSOR_AUV_HPP
#define JOY_PROCESSOR_AUV_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include "joystick_interface_auv/joystick_utils.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick_interface_auv {
class JoyProcessor : {
   public:
    JoyProcessor(State joy_gains, State reference_gains);
    ~JoyProcessor();

    vortex_msgs::msg::ReferenceFilter create_reference_message();
    vortex_msgs::msg::Wrench create_wrench_message();

    void check_number_of_buttons(sensor_msgs::msg::Joy msg);

    std::unordered_map<std::string, int> populate_buttons_map(
        sensor_msgs::msg::Joy msg);
    std::unordered_map<std::string, int> populate_axes_map(
        sensor_msgs::msg::Joy msg);

    void calculate_movement(std::unordered_map<std::string, int> button_map,
                            std::unordered_map<std::string, int> axis_map);

    void update_reference(double dt);

   private:
    State joy_gains_;
    State reference_gains_;
    State drone_state_;
    State reference_state_;

    std::string _mode;

    std::unordered_map<std::string, int> joystick_buttons_map_;
    std::unordered_map<std::string, int> joystick_axes_map_;

    double _last_button_press_time;
    double _last_msg_time;
}
}  // namespace joystick_interface_auv

#endif  // JOY_PROCESSOR_AUV_HPP

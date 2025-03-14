#ifndef JOY_PROCESSOR_AUV_HPP
#define JOY_PROCESSOR_AUV_HPP

#include "unordered_map"
#include "string"
#include <joystick_interface_auv/joystick_utils.hpp>

namespace joystick_interface_auv
{
class JoyProcessor:
{
    public:
        JoyProcessor(State joy_gains, 
                     State reference_gains);
        ~JoyProcessor();

        void calculate_movement(std::unordered_map<std::string, int> button_map,
                                std::unordered_map<std::string, int> axis_map);

    private:
        State joy_gains_;
        State reference_gains_;
        State drone_state_;
        State reference_state_;
        State 
}
}  // namespace joystick_interface_auv

#endif  // JOY_PROCESSOR_AUV_HPP
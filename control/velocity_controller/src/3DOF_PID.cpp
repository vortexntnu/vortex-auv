#include "velocity_controller/lib/3DOF_PID.hpp"
//#include "velocity_controller/lib/PID_controller.hpp"
#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>

PID_3DOF::PID_3DOF(PID_3DOF_params params)
    : surge_controller(params.surge_params), pitch_controller(params.pitch_params), yaw_controller(params.yaw_params) {};

geometry_msgs::msg::WrenchStamped PID_3DOF::calculate_thrust(State state, State error_state) {
    geometry_msgs::msg::WrenchStamped u;
    u.wrench.force.set__x(surge_controller.calculate_thrust(error_state.surge));
    u.wrench.torque.set__y(pitch_controller.calculate_thrust(error_state.pitch,error_state.yaw_rate));
    u.wrench.torque.set__z(yaw_controller.calculate_thrust(error_state.yaw, error_state.yaw_rate));
    if(surge_controller.get_validity() && pitch_controller.get_validity() && yaw_controller.get_validity()){
        valid = true;
    } else {
        valid = false;
    }
    return u;
}

void PID_3DOF::reset_controller(int nr){
    switch (nr){
    case 0:
        surge_controller.reset_controller();
        pitch_controller.reset_controller();
        yaw_controller.reset_controller();
        break;
    case 1:
        surge_controller.reset_controller();
        break;
    case 2:
        pitch_controller.reset_controller();
        break;
    case 3:
        yaw_controller.reset_controller();
        break;
    }
    return;
}
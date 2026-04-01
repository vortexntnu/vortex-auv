#include "velocity_controller/control_manager.hpp"
#include "velocity_controller/lib/3DOF_PID.hpp"
#include "velocity_controller/lib/controller.hpp"
#include "vortex_msgs/msg/los_guidance.hpp"
#include "vortex/utils/math.hpp"

control_manager::control_manager(control_manager_params params) : params_(params) {}

bool control_manager::switch_controller() {
    // Implementation for switching controllers
    return true;
}

void control_manager::shutdown_controller() {
    controller_3DOF.reset();
    controller_LQR.reset();
}

geometry_msgs::msg::WrenchStamped control_manager::get_output(Guidance_data guidance_values, State current_state) {
    // TODO(henrimha): Do I need ssa here?
    angle ref_in_body =
        angle_NED_to_body({0, vortex::utils::math::ssa(guidance_values.pitch),
                           vortex::utils::math::ssa(guidance_values.yaw)},
                          current_state.get_angle());
    State error_state_body;
    error_state_body.surge = guidance_values.surge - current_state.surge;
    error_state_body.pitch = -ref_in_body.thetat;
    error_state_body.yaw = -ref_in_body.psit;

    if (params_.anti_overshoot) {
        if (abs(error_state_body.yaw) < std::numbers::pi / 2 ||
            abs(error_state_body.pitch) < std::numbers::pi / 2) {
            error_state_body.surge =
                guidance_values.surge * cos(error_state_body.yaw) * cos(error_state_body.pitch);
        }
    }
    switch (params_.control_type) {
        case 1:
            return controller_3DOF->calculate_thrust(current_state, error_state_body);
        case 2:
            return controller_LQR->calculate_thrust(current_state, error_state_body);
        default:
            return geometry_msgs::msg::WrenchStamped();
    }
}
void control_manager::initialize_3DOF_controller(PID_3DOF_params params) {
    controller_3DOF = std::make_unique<PID_3DOF>(params);
}

void control_manager::initialize_LQR_controller(LQR_params params) {
    controller_LQR = std::make_unique<LQRController>(params);
}

bool control_manager::get_validity() {
    switch (params_.control_type) {
        case 1:
            return controller_3DOF->get_validity();
        case 2:
            return controller_LQR->get_validity();
        default:
            return false;
    }
}

void control_manager::reset_controllers(int nr) {
    switch (params_.control_type) {
        case 1:
            controller_3DOF->reset_controller(nr);
            break;
        case 2:
            controller_LQR->reset_controller(nr);
            break;
    }
}
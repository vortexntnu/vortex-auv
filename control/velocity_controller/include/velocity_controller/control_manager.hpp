#ifndef VELOCITY_CONTROLLER__CONTROL_MANAGER_HPP_
#define VELOCITY_CONTROLLER__CONTROL_MANAGER_HPP_
#include "velocity_controller/lib/controller.hpp"
#include "velocity_controller/utilities.hpp"
#include "velocity_controller/lib/3DOF_PID.hpp"
#include "velocity_controller/lib/LQR_setup.hpp"
struct control_manager_params{
    int control_type; // 1 3DOF PID, 2 3DOF LQR
    bool anti_overshoot;
    bool fallback;
};
class control_manager{
    public:
    control_manager(control_manager_params params);
    bool switch_controller();
    void shutdown_controller();
    geometry_msgs::msg::WrenchStamped get_output(Guidance_data guidanceState, State State);
    void initialize_3DOF_controller(PID_3DOF_params params);
    void initialize_LQR_controller(LQR_params params);
    bool get_validity();
    void reset_controllers(int nr = 0);
    private:
    control_manager_params params_;
    std::unique_ptr<PID_3DOF> controller_3DOF=nullptr;
    std::unique_ptr<LQRController> controller_LQR=nullptr;
    

};
#endif // VELOCITY_CONTROLLER__CONTROL_MANAGER_HPP_
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
    control_manager_params(int control_type_val, bool anti_overshoot_val, bool fallback_val)
        : control_type(control_type_val), anti_overshoot(anti_overshoot_val), fallback(fallback_val) {}
    control_manager_params() = default;
    controller_params control_params;
};
/** @brief The class that is the interface between ros and the control logic for the velocity controller.
    It handles the selection and switching of different controllers (3DOF PID or LQR) based on the provided parameters, reseting and more.
    The control_manager class is responsible for initializing the appropriate controller, calculating the required thrust based on the current state and guidance data, and managing the validity of the controller's output.
*/
class control_manager{
    public:
    control_manager(control_manager_params params);
    /** @brief Switches the active controller.
        @return true if the controller was successfully switched, false otherwise.
    */
    bool switch_controller(int nr);

    /** @brief Gets the output from the active controller.
        @param guidanceState: The current guidance data.
        @param State: The current state of the vehicle.
        @return The calculated thrust output as a WrenchStamped message.
    */
    geometry_msgs::msg::WrenchStamped get_output(Guidance_data guidanceState, State State);
    void initialize_3DOF_controller(PID_3DOF_params params);
    void initialize_LQR_controller(LQR_params params);
    /** @brief Gets the validity of the active controller.
        @return true if the output of the active controller is valid, false otherwise.
    */
    bool get_validity();
    /** @brief Resets the active controller.
        @param nr: optional parameter to reset specific parts of the controller (for example surge), default is 0 which resets all dimensions
    */
    void reset_controller(int nr = 0);
    /** @brief Resets all controllers.
        This function resets all controllers managed by the control_manager, clearing any internal states and errors.
    */
    void reset_controllers();
    private:
    control_manager_params params_;
    std::unique_ptr<PID_3DOF> controller_3DOF=nullptr;
    std::unique_ptr<LQRController> controller_LQR=nullptr;
    

};
#endif // VELOCITY_CONTROLLER__CONTROL_MANAGER_HPP_
#ifndef VELOCITY_CONTROLLER__3DOF_PID_HPP_
#define VELOCITY_CONTROLLER__3DOF_PID_HPP_

#include "velocity_controller/lib/controller.hpp"
#include "velocity_controller/utilities.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "velocity_controller/lib/PID_controller.hpp"
struct PID_3DOF_params{
    PID_params surge_params;
    PID_params pitch_params;
    PID_params yaw_params;
};
class PID_3DOF : public controller {
   public:
    PID_3DOF(PID_3DOF_params params);
    geometry_msgs::msg::WrenchStamped calculate_thrust(State state, State error_state) override;
    void reset_controller(int nr=0) override;
    ///bool set_parameters(PID_params surge_params, PID_params pitch_params, PID_params yaw_params);

   private:
    PID_controller surge_controller;
    PID_controller pitch_controller;
    PID_controller yaw_controller;
};


#endif // VELOCITY_CONTROLLER__3DOF_PID_HPP_
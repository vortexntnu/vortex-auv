#ifndef VELOCITY_CONTROLLER__CONTROLLER_HPP_
#define VELOCITY_CONTROLLER__CONTROLLER_HPP_
#include <memory>
#include "velocity_controller/utilities.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class controller{
    public:
    controller()=default;
    virtual geometry_msgs::msg::WrenchStamped calculate_thrust(State state, State error_state) = 0;
    virtual void reset_controller(int nr=0) = 0;
    bool get_validity(){return valid;};

    protected:
    bool valid=false;
};



#endif // VELOCITY_CONTROLLER__CONTROLLER_HPP_
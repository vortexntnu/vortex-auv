#ifndef CONTROLLER_TEST_ACCESSOR_HPP_
#define CONTROLLER_TEST_ACCESSOR_HPP_
#include "velocity_controller/lib/controller.hpp"

class ControllerTestAccessor {
public:
    static Eigen::Vector<double, 6> get_tau_max(const controller& c) {
        return c.tau_max;
    }
    static bool get_saturated(const controller& c, int index) {
        return c.saturated[index];
    }
};
#endif
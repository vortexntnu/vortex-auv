#pragma once

#include "vortex/utils/types.hpp"

namespace vortex::runtime::vehicle_control {

struct RuntimeState {
    bool running = false;
    bool killswitch_on = true;

    vortex::utils::types::Mode operation_mode =
        vortex::utils::types::Mode::manual;
};

}  // namespace vortex::runtime::vehicle_control

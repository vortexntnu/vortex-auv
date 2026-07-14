#pragma once

#include "vehicle_config.hpp"

namespace vortex::runtime::vehicle_control::config {

VehicleConfig make_nautilus_config(
    std::chrono::milliseconds control_period
);

}

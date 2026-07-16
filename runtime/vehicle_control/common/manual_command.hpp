#pragma once

#include <Eigen/Core>

namespace vortex::runtime::vehicle_control {

using WrenchVector = Eigen::Matrix<double, 6, 1>;

struct ManualCommand {
    WrenchVector wrench = WrenchVector::Zero();
    bool active = false;
};

}  // namespace vortex::runtime::vehicle_control

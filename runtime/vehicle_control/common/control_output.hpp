#pragma once

#include <Eigen/Core>

namespace vortex::runtime::vehicle_control {

constexpr Eigen::Index thruster_count = 8;

using ThrusterForces =
    Eigen::Matrix<double, thruster_count, 1>;

struct ControlOutput {
    /*
     * False means that the output layer must command zero thrust or
     * disable the thruster interface.
     */
    bool thrusters_enabled = false;

    ThrusterForces forces_n = ThrusterForces::Zero();
};

}  // namespace vortex::runtime::vehicle_control

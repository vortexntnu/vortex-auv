#pragma once

#include "common/control_output.hpp"
#include "common/sensor_frame.hpp"

#include <string>

namespace vortex::simulation::stonefish {
class VortexSimulationManager;
}

namespace vortex::runtime::vehicle_control {

struct StonefishIoConfig {
    std::string imu_sensor_name =
        "nautilus/imu_link";

    std::string pressure_sensor_name =
        "nautilus/pressure_sensor_link";

    std::string dvl_sensor_name =
        "nautilus/dvl_link";

    /*
     * Maximum absolute force represented by one simulator thruster
     * command.
     */
    double max_thruster_force_n = 0.0;
};

class StonefishIo {
public:
    explicit StonefishIo(StonefishIoConfig config);

    [[nodiscard]]
    SensorFrame read_sensors(
        vortex::simulation::stonefish::VortexSimulationManager& manager)
        const;

    void apply_thrusters(
        vortex::simulation::stonefish::VortexSimulationManager& manager,
        const ControlOutput& output) const;

    void stop_thrusters(
        vortex::simulation::stonefish::VortexSimulationManager& manager)
        const;

private:
    StonefishIoConfig config_;
};

}  // namespace vortex::runtime::vehicle_control

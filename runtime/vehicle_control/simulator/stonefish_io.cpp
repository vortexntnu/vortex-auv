#include "simulator/stonefish_io.hpp"

#include "simulation/stonefish/vortex_simulation_manager.hpp"
#include "simulation/stonefish/thruster_command.hpp"

#include "simulator/make_sim_thruster_command.hpp"

#include <tracy/Tracy.hpp>

#include <chrono>
#include <stdexcept>
#include <utility>

namespace vortex::runtime::vehicle_control {

StonefishIo::StonefishIo(StonefishIoConfig config)
    : config_{std::move(config)}
{
    if (config_.imu_sensor_name.empty()) {
        throw std::invalid_argument{
            "Stonefish IMU sensor name cannot be empty"};
    }

    if (config_.pressure_sensor_name.empty()) {
        throw std::invalid_argument{
            "Stonefish pressure sensor name cannot be empty"};
    }

    if (config_.dvl_sensor_name.empty()) {
        throw std::invalid_argument{
            "Stonefish DVL sensor name cannot be empty"};
    }

    if (config_.max_thruster_force_n <= 0.0) {
        throw std::invalid_argument{
            "Stonefish maximum thruster force must be positive"};
    }
}

SensorFrame StonefishIo::read_sensors(
    vortex::simulation::stonefish::VortexSimulationManager& manager)
    const
{
    ZoneScopedN("Read Stonefish sensors");

    SensorFrame frame;
    const auto timestamp = std::chrono::steady_clock::now();

    {
        ZoneScopedN("Read Stonefish IMU");

        const auto imu =
            manager.read_imu(config_.imu_sensor_name);

        if (imu.valid) {
            frame.imu = ImuSample{
                .linear_acceleration_m_s2 =
                    Eigen::Vector3d{
                        imu.linear_acceleration_m_s2[0],
                        imu.linear_acceleration_m_s2[1],
                        imu.linear_acceleration_m_s2[2],
                    },

                .angular_velocity_rad_s =
                    Eigen::Vector3d{
                        imu.angular_velocity_rad_s[0],
                        imu.angular_velocity_rad_s[1],
                        imu.angular_velocity_rad_s[2],
                    },

                .timestamp = timestamp,
            };
        }
    }

    {
        ZoneScopedN("Read Stonefish pressure");

        const auto pressure =
            manager.read_pressure(
                config_.pressure_sensor_name);

        if (pressure.valid) {
            frame.depth = DepthSample{
                .depth_m = pressure.depth_m,
                .timestamp = timestamp,
            };
        }
    }

    {
        ZoneScopedN("Read Stonefish DVL");

        const auto dvl =
            manager.read_dvl(config_.dvl_sensor_name);

        if (dvl.valid) {
            frame.dvl = DvlSample{
                .velocity_body_m_s =
                    Eigen::Vector3d{
                        dvl.velocity_body_m_s[0],
                        dvl.velocity_body_m_s[1],
                        dvl.velocity_body_m_s[2],
                    },

                .timestamp = timestamp,
            };
        }
    }

    return frame;
}

void StonefishIo::apply_thrusters(
    vortex::simulation::stonefish::VortexSimulationManager& manager,
    const ControlOutput& output) const
{
    ZoneScopedN("Apply Stonefish thrusters");

    if (!output.thrusters_enabled) {
        stop_thrusters(manager);
        return;
    }

    const auto command =
        make_sim_thruster_command(
            output.forces_n,
            config_.max_thruster_force_n);

    manager.set_thrusters(command);
}

void StonefishIo::stop_thrusters(
    vortex::simulation::stonefish::VortexSimulationManager& manager)
    const
{
    ZoneScopedN("Stop Stonefish thrusters");

    manager.set_thrusters(
        vortex::simulation::stonefish::ThrusterCommand{});
}

}  // namespace vortex::runtime::vehicle_control

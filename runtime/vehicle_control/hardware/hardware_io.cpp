#include "hardware/hardware_io.hpp"

#include "hardware/drivers/dvl_driver.hpp"
#include "hardware/drivers/imu_driver.hpp"
#include "hardware/drivers/pressure_driver.hpp"
#include "hardware/drivers/thruster_driver.hpp"

#include <tracy/Tracy.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vortex::runtime::vehicle_control {

HardwareIo::HardwareIo(
    HardwareIoConfig config,
    ImuDriver& imu_driver,
    DvlDriver& dvl_driver,
    PressureDriver& pressure_driver,
    ThrusterDriver& thruster_driver)
    : config_{config},
      imu_driver_{imu_driver},
      dvl_driver_{dvl_driver},
      pressure_driver_{pressure_driver},
      thruster_driver_{thruster_driver},
      last_valid_command_time_{
          std::chrono::steady_clock::now()}
{
    if (config_.command_timeout <=
        std::chrono::milliseconds::zero()) {
        throw std::invalid_argument{
            "Hardware command timeout must be positive"};
    }

    if (config_.max_thruster_force_n <= 0.0) {
        throw std::invalid_argument{
            "Hardware maximum thruster force must be positive"};
    }
}

SensorFrame HardwareIo::read_sensors()
{
    ZoneScopedN("Read hardware sensors");

    SensorFrame frame;

    {
        ZoneScopedN("Read hardware IMU");

        if (const auto imu = imu_driver_.take_latest()) {
            frame.imu = ImuSample{
                .linear_acceleration_m_s2 =
                    imu->linear_acceleration_m_s2,

                .angular_velocity_rad_s =
                    imu->angular_velocity_rad_s,

                .timestamp = imu->timestamp,
            };
        }
    }

    {
        ZoneScopedN("Read hardware DVL");

        if (const auto dvl = dvl_driver_.take_latest()) {
            frame.dvl = DvlSample{
                .velocity_body_m_s =
                    dvl->velocity_body_m_s,

                .timestamp = dvl->timestamp,
            };
        }
    }

    {
        ZoneScopedN("Read hardware pressure");

        if (const auto pressure =
                pressure_driver_.take_latest()) {
            frame.depth = DepthSample{
                .depth_m = pressure->depth_m,
                .timestamp = pressure->timestamp,
            };
        }
    }

    return frame;
}

bool HardwareIo::apply_thrusters(
    const ControlOutput& output)
{
    ZoneScopedN("Apply hardware thrusters");

    if (!output.thrusters_enabled) {
        return stop_thrusters();
    }

    const ThrusterForces clamped_forces =
        clamp_forces(output.forces_n);

    if (!thrusters_enabled_) {
        if (!thruster_driver_.enable()) {
            thrusters_enabled_ = false;
            return false;
        }

        thrusters_enabled_ = true;
    }

    if (!thruster_driver_.set_forces(clamped_forces)) {
        /*
         * A failed command should place the output in a safe state.
         */
        static_cast<void>(stop_thrusters());
        return false;
    }

    last_valid_command_time_ =
        std::chrono::steady_clock::now();

    return true;
}

bool HardwareIo::stop_thrusters()
{
    ZoneScopedN("Stop hardware thrusters");

    /*
     * Send zero first so a driver that does not immediately remove
     * power still receives a neutral command.
     */
    const bool zero_command_sent =
        thruster_driver_.set_forces(
            ThrusterForces::Zero());

    const bool disabled =
        thruster_driver_.disable();

    thrusters_enabled_ = false;

    return zero_command_sent && disabled;
}

void HardwareIo::update_watchdog()
{
    ZoneScopedN("Hardware I/O watchdog");

    if (!thrusters_enabled_) {
        return;
    }

    const auto now =
        std::chrono::steady_clock::now();

    if (now - last_valid_command_time_ >
        config_.command_timeout) {
        static_cast<void>(stop_thrusters());
    }
}

ThrusterForces HardwareIo::clamp_forces(
    const ThrusterForces& forces_n) const
{
    ThrusterForces clamped = forces_n;

    for (Eigen::Index index = 0;
         index < clamped.size();
         ++index) {
        double& force = clamped[index];

        if (!std::isfinite(force)) {
            force = 0.0;
            continue;
        }

        force = std::clamp(
            force,
            -config_.max_thruster_force_n,
            config_.max_thruster_force_n);
    }

    return clamped;
}

}  // namespace vortex::runtime::vehicle_control

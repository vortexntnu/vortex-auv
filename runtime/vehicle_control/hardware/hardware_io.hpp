#pragma once

#include "common/control_output.hpp"
#include "common/sensor_frame.hpp"

#include <chrono>

namespace vortex::runtime::vehicle_control {

// temp drivers

struct HardwareImuReading {
    Eigen::Vector3d linear_acceleration_m_s2;
    Eigen::Vector3d angular_velocity_rad_s;
    std::chrono::steady_clock::time_point timestamp;
};

class ImuDriver {
public:
    std::optional<HardwareImuReading> take_latest();
};

struct HardwareDvlReading {
    Eigen::Vector3d velocity_body_m_s;
    std::chrono::steady_clock::time_point timestamp;
};

class DvlDriver {
public:
    std::optional<HardwareDvlReading> take_latest();
};

struct HardwarePressureReading {
    double depth_m;
    std::chrono::steady_clock::time_point timestamp;
};

class PressureDriver {
public:
    std::optional<HardwarePressureReading> take_latest();
};

class ThrusterDriver {
public:
    bool enable();
    bool disable();
    bool set_forces(const ThrusterForces& forces_n);
};




class ImuDriver;
class DvlDriver;
class PressureDriver;
class ThrusterDriver;

struct HardwareIoConfig {
    /*
     * If no valid control output is applied for this period, the
     * hardware I/O layer disables the thrusters.
     *
     * The MCU should still have its own independent watchdog.
     */
    std::chrono::milliseconds command_timeout{
        200};

    /*
     * Clamp commands before sending them to the thruster interface.
     */
    double max_thruster_force_n = 0.0;
};

class HardwareIo {
public:
    HardwareIo(
        HardwareIoConfig config,
        ImuDriver& imu_driver,
        DvlDriver& dvl_driver,
        PressureDriver& pressure_driver,
        ThrusterDriver& thruster_driver);

    HardwareIo(const HardwareIo&) = delete;
    HardwareIo& operator=(const HardwareIo&) = delete;

    [[nodiscard]]
    SensorFrame read_sensors();

    [[nodiscard]]
    bool apply_thrusters(const ControlOutput& output);

    [[nodiscard]]
    bool stop_thrusters();

    /*
     * Call this during every hardware-loop iteration, including when
     * no sensor measurements are available.
     */
    void update_watchdog();

private:
    [[nodiscard]]
    ThrusterForces clamp_forces(
        const ThrusterForces& forces_n) const;

    HardwareIoConfig config_;

    ImuDriver& imu_driver_;
    DvlDriver& dvl_driver_;
    PressureDriver& pressure_driver_;
    ThrusterDriver& thruster_driver_;

    bool thrusters_enabled_ = false;

    std::chrono::steady_clock::time_point
        last_valid_command_time_;
};

}  // namespace vortex::runtime::vehicle_control

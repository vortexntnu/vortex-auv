#pragma once

#include <Eigen/Core>

#include <chrono>
#include <optional>

namespace vortex::runtime::vehicle_control {

using SteadyTimePoint = std::chrono::steady_clock::time_point;

struct ImuSample {
    Eigen::Vector3d angular_velocity_rad_s =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d linear_acceleration_m_s2 =
        Eigen::Vector3d::Zero();

    SteadyTimePoint timestamp{};
};

struct DvlSample {
    Eigen::Vector3d velocity_m_s =
        Eigen::Vector3d::Zero();

    double altitude_m = 0.0;
    double velocity_quality = 0.0;

    bool velocity_valid = false;
    bool altitude_valid = false;

    SteadyTimePoint timestamp{};
};

struct DepthSample {
    double pressure_pa = 0.0;
    double temperature_c = 0.0;
    double depth_m = 0.0;

    SteadyTimePoint timestamp{};
};

struct SensorFrame {
    std::optional<ImuSample> imu;
    std::optional<DvlSample> dvl;
    std::optional<DepthSample> depth;
};

}  // namespace vortex::runtime::vehicle_control

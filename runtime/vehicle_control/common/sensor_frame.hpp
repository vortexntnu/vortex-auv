#pragma once

#include <Eigen/Core>

#include <chrono>
#include <optional>

namespace vortex::runtime::vehicle_control {

using SteadyTimePoint = std::chrono::steady_clock::time_point;

struct ImuSample {
    Eigen::Vector3d linear_acceleration_m_s2 =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d angular_velocity_rad_s =
        Eigen::Vector3d::Zero();

    SteadyTimePoint timestamp{};
};

struct DvlSample {
    Eigen::Vector3d velocity_body_m_s =
        Eigen::Vector3d::Zero();

    SteadyTimePoint timestamp{};
};

struct DepthSample {
    double depth_m = 0.0;

    SteadyTimePoint timestamp{};
};

struct SensorFrame {
    /*
     * A populated optional means that a new measurement is available.
     *
     * This prevents slower sensors, such as the DVL, from being applied
     * repeatedly during every control-loop iteration.
     */
    std::optional<ImuSample> imu;
    std::optional<DvlSample> dvl;
    std::optional<DepthSample> depth;
};

}  // namespace vortex::runtime::vehicle_control

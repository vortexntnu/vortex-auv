#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace vortex::simulation::stonefish {

struct ImuReading {
    double timestamp_s{0.0};

    std::array<double, 3> angular_velocity_rad_s{};
    std::array<double, 3> linear_acceleration_m_s2{};
    std::array<double, 4> orientation_xyzw{};  // x, y, z, w

    bool valid{false};
};

struct PressureReading {
    double timestamp_s{0.0};

    double pressure_pa{0.0};
    double depth_m{0.0};

    bool valid{false};
};

struct DvlReading {
    double timestamp_s{0.0};

    std::array<double, 3> velocity_body_m_s{};
    std::array<bool, 4> beam_valid{};

    double altitude_m{0.0};

    bool velocity_valid{false};
    bool altitude_valid{false};
    bool valid{false};
};

struct CameraFrame {
    double timestamp_s{0.0};

    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t channels{0};

    std::vector<std::uint8_t> pixels;

    bool valid{false};
};

struct SonarFrame {
    double timestamp_s{0.0};

    std::uint32_t beams{0};
    std::uint32_t bins{0};

    std::vector<float> intensities;

    bool valid{false};
};

}  // namespace vortex::simulation::stonefish

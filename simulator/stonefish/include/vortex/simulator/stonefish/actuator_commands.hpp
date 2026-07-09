#pragma once

#include <array>
#include <cstddef>

namespace vortex::simulation::stonefish {

struct ThrusterCommand {
    static constexpr std::size_t kNumThrusters = 8;

    std::array<double, kNumThrusters> command{};
};

}  // namespace vortex::simulation::stonefish

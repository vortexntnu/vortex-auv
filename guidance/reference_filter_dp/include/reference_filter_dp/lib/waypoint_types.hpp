#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

#include <cstdint>
#include <vortex/utils/types.hpp>

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;

enum class WaypointMode : uint8_t {
    FULL_POSE = 0,
    ONLY_POSITION = 1,
    FORWARD_HEADING = 2,
    ONLY_ORIENTATION = 3,
};

struct Waypoint {
    PoseEuler pose{};
    WaypointMode mode = WaypointMode::FULL_POSE;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

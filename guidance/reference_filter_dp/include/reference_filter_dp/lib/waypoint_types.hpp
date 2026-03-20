#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

#include <cstdint>
#include "reference_filter_dp/lib/eigen_typedefs.hpp"

namespace vortex::guidance {

enum class WaypointMode : uint8_t {
    FULL_POSE = 0,
    ONLY_POSITION = 1,
    FORWARD_HEADING = 2,
    ONLY_ORIENTATION = 3,
};

struct Waypoint {
    Eigen::Vector6d pose = Eigen::Vector6d::Zero();
    WaypointMode mode = WaypointMode::FULL_POSE;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

#include <vortex/utils/types.hpp>

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;
using vortex::utils::types::WaypointMode;

/**
 * @brief A target pose with an associated waypoint mode.
 *
 */
struct WaypointEuler {
    PoseEuler pose{};
    WaypointMode mode = WaypointMode::FULL_POSE;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

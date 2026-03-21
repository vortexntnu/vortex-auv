#ifndef REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_TYPES_HPP_
#define REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_TYPES_HPP_

#include <cstdint>
#include <vortex/utils/types.hpp>

namespace vortex::guidance {

using vortex::utils::types::Pose;

/**
 * @brief Determines which degrees of freedom the reference filter controls.
 *
 * The mode affects both the reference goal computation (via apply_mode_logic)
 * and the convergence check (via has_converged).
 */
enum class WaypointMode : uint8_t {
    FULL_POSE = 0,         ///< Control all 6 DOF.
    ONLY_POSITION = 1,     ///< Control x, y, z; hold current orientation.
    FORWARD_HEADING = 2,   ///< Control x, y, z with yaw toward target.
    ONLY_ORIENTATION = 3,  ///< Control roll, pitch, yaw; hold current position.
};

/**
 * @brief A target pose with an associated waypoint mode.
 */
struct Waypoint {
    Pose pose{};
    WaypointMode mode = WaypointMode::FULL_POSE;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_TYPES_HPP_

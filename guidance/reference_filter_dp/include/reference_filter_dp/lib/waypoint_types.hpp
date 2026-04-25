#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

#include <cstdint>
#include <vortex/utils/types.hpp>

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;

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
    POSITION_AND_YAW = 4,  ///< Control x, y, z and yaw; force roll=pitch=0.
    XY_AND_YAW = 5,  ///< Control x, y and yaw; hold z, force roll=pitch=0.
};

/**
 * @brief A target pose with an associated waypoint mode.
 */
struct Waypoint {
    PoseEuler pose{};
    WaypointMode mode = WaypointMode::FULL_POSE;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

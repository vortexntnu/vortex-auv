#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

#include <cstdint>
#include <vortex/utils/types.hpp>
#include <vortex/utils/waypoint_utils.hpp>

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;
using vortex::utils::waypoints::ControlledError;
using vortex::utils::waypoints::ConvergenceTolerance;

/**
 * @brief Determines which degrees of freedom the reference filter controls.
 *
 * The mode affects both the reference goal computation (via apply_mode_logic)
 * and the convergence check (via has_converged).
 *
 * The values match the constants in vortex_msgs/WaypointMode so a mode reads
 * the same in logs and messages. XY_FORWARD_DIR (6), LEVEL_ORIENTATION (7)
 * and POS_Z_LEVEL_ORIENTATION (9) are not supported by this filter, since
 * FORWARD_HEADING, ONLY_ORIENTATION and ONLY_Z cover them.
 */
enum class WaypointMode : uint8_t {
    FULL_POSE = 0,         ///< Control all 6 DOF.
    ONLY_POSITION = 1,     ///< Control x, y, z; hold current orientation.
    FORWARD_HEADING = 2,   ///< Control x, y, z with yaw toward target.
    ONLY_ORIENTATION = 3,  ///< Control roll, pitch, yaw; hold current position.
    POSITION_AND_YAW = 4,  ///< Control x, y, z and yaw; force roll=pitch=0.
    XY_AND_YAW = 5,  ///< Control x, y and yaw; hold z, force roll=pitch=0.
    ONLY_Z = 8,      ///< Hold current x, y, orientation; control z only.
};

/**
 * @brief A target pose with an associated waypoint mode.
 */
struct Waypoint {
    PoseEuler pose{};
    WaypointMode mode = WaypointMode::FULL_POSE;
    bool keep_altitude{false};
    double desired_altitude{0.0};
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_TYPES_HPP_

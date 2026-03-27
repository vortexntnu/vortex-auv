#ifndef REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_UTILS_HPP_
#define REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_UTILS_HPP_

#include "reference_filter_dp_quat/lib/waypoint_types.hpp"

namespace vortex::guidance {

using vortex::utils::types::Pose;

/**
 * @brief Compute the waypoint goal by applying the mode logic to the incoming
 * waypoint.
 *
 * For modes that don't control all DOFs, the uncontrolled components are
 * replaced with values from @p current_state.
 *
 * @param incoming_waypoint The incoming waypoint to compute goal from.
 * @param mode The waypoint mode.
 * @param current_state The current state pose.
 * @return The adjusted waypoint goal.
 */
Pose compute_waypoint_goal(const Pose& incoming_waypoint,
                           WaypointMode mode,
                           const Pose& current_state);

/**
 * @brief Check whether the state has converged to the waypoint goal.
 *
 * Only the DOFs relevant to the waypoint mode are included in the error norm.
 *
 * @param state The current state pose.
 * @param waypoint_goal The waypoint goal pose.
 * @param mode The waypoint mode.
 * @param convergence_threshold The maximum allowed error norm.
 * @return True if the error is below the threshold.
 */
bool has_converged(const Pose& state,
                   const Pose& waypoint_goal,
                   WaypointMode mode,
                   double convergence_threshold);

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_UTILS_HPP_

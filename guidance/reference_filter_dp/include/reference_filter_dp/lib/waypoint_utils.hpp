#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_UTILS_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_UTILS_HPP_

#include "reference_filter_dp/lib/eigen_typedefs.hpp"
#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

/**
 * @brief Apply waypoint mode logic to a reference pose.
 *
 * For modes that don't control all DOFs, the uncontrolled components are
 * replaced with values from @p current_state so the filter holds them steady.
 *
 * @param r_in The raw reference pose (6D).
 * @param mode The waypoint mode.
 * @param current_state The current filter state pose (6D).
 * @return The adjusted reference pose.
 */
Eigen::Vector6d apply_mode_logic(const Eigen::Vector6d& r_in,
                                 WaypointMode mode,
                                 const Eigen::Vector6d& current_state);

/**
 * @brief Check whether the measured pose has converged to the reference.
 *
 * Only the DOFs relevant to the waypoint mode are included in the error norm.
 *
 * @param measured_pose The current measured pose (6D).
 * @param reference The reference goal pose (6D).
 * @param mode The waypoint mode.
 * @param convergence_threshold The maximum allowed error norm.
 * @return True if the error is below the threshold.
 */
bool has_converged(const Eigen::Vector6d& measured_pose,
                   const Eigen::Vector6d& reference,
                   WaypointMode mode,
                   double convergence_threshold);

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_UTILS_HPP_

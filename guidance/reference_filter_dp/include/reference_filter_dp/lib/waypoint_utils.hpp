#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_UTILS_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_UTILS_HPP_

#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

Eigen::Vector6d apply_mode_logic(const Eigen::Vector6d& r_in,
                                 WaypointMode mode,
                                 const Eigen::Vector6d& current_state);

bool has_converged(const Eigen::Vector6d& measured_pose,
                   const Eigen::Vector6d& reference,
                   WaypointMode mode,
                   double convergence_threshold);

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_UTILS_HPP_

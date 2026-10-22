#ifndef WAYPOINT_MANAGER__FRAME_RESOLVER_HPP_
#define WAYPOINT_MANAGER__FRAME_RESOLVER_HPP_

#include <vortex/utils/types.hpp>

namespace vortex::mission {

using vortex::utils::types::Pose;

enum class GoalFrame { WORLD, BODY_RELATIVE, WORLD_RELATIVE };

/**
 * @brief Resolve a waypoint pose to an absolute pose in odom.
 * @param target Absolute pose (WORLD) or offset from @p start.
 * @param frame How @p target is interpreted.
 * @param start Vehicle pose in odom at goal start.
 */
Pose resolve_pose(const Pose& target, GoalFrame frame, const Pose& start);

}  // namespace vortex::mission

#endif  // WAYPOINT_MANAGER__FRAME_RESOLVER_HPP_

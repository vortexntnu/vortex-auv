#ifndef WAYPOINT_MANAGER__FRAME_RESOLVER_HPP_
#define WAYPOINT_MANAGER__FRAME_RESOLVER_HPP_

#include <vortex/utils/types.hpp>

namespace vortex::mission {

using vortex::utils::types::Pose;

/**
 * @brief How the poses in a waypoint goal are to be interpreted.
 */
enum class OffsetFrame {
    /// Absolute poses in odom.
    WORLD,
    /// Offsets in the vehicle frame at goal start (x forward, y right, z down).
    BODY_RELATIVE,
    /// Offsets along the odom axes, relative to the vehicle pose at goal start.
    WORLD_RELATIVE
};

/**
 * @brief Resolve a waypoint pose to an absolute pose in odom.
 *
 * - WORLD: returned unchanged.
 * - BODY_RELATIVE: p = p0 + R0 * p_offset, q = q0 * q_offset
 * - WORLD_RELATIVE: p = p0 + p_offset, q = q_offset * q0
 *
 * where (p0, q0) is @p start, the vehicle pose in odom when the goal started.
 *
 * @param target The waypoint pose (absolute or an offset, depending on frame).
 * @param frame How @p target is to be interpreted.
 * @param start The vehicle pose in odom at goal start.
 * @return The absolute pose in odom.
 */
Pose resolve_pose(const Pose& target, OffsetFrame frame, const Pose& start);

}  // namespace vortex::mission

#endif  // WAYPOINT_MANAGER__FRAME_RESOLVER_HPP_

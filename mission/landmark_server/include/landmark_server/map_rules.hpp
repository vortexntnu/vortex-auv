#ifndef LANDMARK_SERVER__MAP_RULES_HPP_
#define LANDMARK_SERVER__MAP_RULES_HPP_

#include <eigen3/Eigen/Dense>
#include "landmark_server/class_config.hpp"
#include "landmark_server/course_frame.hpp"
#include "landmark_server/retained_landmarks.hpp"

namespace vortex::mission {

/**
 * @brief Map rules after RetainedLandmarks. ROS-free.
 *
 * Perception gives positions without orientation. The rules derive structure
 * from the parts:
 *  - gate: yaw from the line between the two panels (normal on the line, the
 *    front is the side the vehicle first saw it from), the gate is pulled to
 *    the panel midpoint, a synthetic GATE_WHOLE is made if only the panels
 *    have been seen, and the panels inherit the yaw;
 *  - torpedo board: yaw and centre from the icon pairs, board version from the
 *    icon heights (fire above blood = version 1) and TORPEDO_TARGET_* openings
 *    from icon + offset in the board frame;
 *  - bins: the role icon seen by the down camera gives the role of the nearest
 *    bin;
 *  - octagon: OCTAGON_WHOLE above the table.
 *
 * Yaw is locked after N consistent estimates and never flips afterwards
 * (a yaw that follows the observer would turn when the object is seen from
 * behind).
 * Landmarks made by a rule are marked `derived`. Once the gate yaw is
 * consistent, the estimates are also given to the course frame.
 *
 * Landmark frame: origin in the object, +X out of the front, +Z down (NED).
 *
 * @param map The map after RetainedLandmarks::update().
 * @param config Rules.
 * @param vehicle_position Vehicle position in odom (defines the front on the
 * first estimate).
 * @param now Time [s].
 * @param course Course frame that gets the gate estimates (may be null).
 */
void apply_map_rules(RetainedLandmarks& map,
                     const MapRulesConfig& config,
                     const Eigen::Vector3d& vehicle_position,
                     double now,
                     CourseFrameTracker* course = nullptr);

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__MAP_RULES_HPP_

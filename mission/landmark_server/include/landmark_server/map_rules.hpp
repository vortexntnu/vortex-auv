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
 *    have been seen, and the panels inherit the yaw. Panels farther apart than
 *    max_panel_separation_m are not one gate;
 *  - torpedo board: icons farther than board_icon_radius_m from the board are
 *    ignored; yaw and centre from the icon pairs (yaw only within
 *    board_yaw_max_distance_m), board version from the icon heights (fire
 *    above blood or firetruck above ambulance = version 1, both pairs must
 *    agree, locked after board_version_lock_votes votes) and TORPEDO_TARGET_*
 *    openings from icon + offset in the board frame;
 *  - bins: the role icon seen by the down camera gives the role of the nearest
 *    bin;
 *  - table and octagon: one xy for both, from table_octagon_primary (table,
 *    octagon or midpoint); the missing one is derived from the other (a
 *    table only with z_lock on, at floor_z - table_height_m).
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

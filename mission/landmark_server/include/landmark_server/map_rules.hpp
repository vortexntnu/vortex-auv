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
 * What a fixed offset in a course template cannot say (the gate and the
 * torpedo board come from their templates):
 *  - course frame: while both gate panels are being seen, the line between
 *    them gives the course frame an estimate (the front faces the vehicle
 *    that first saw it). Panels closer than min_panel_separation_m or
 *    farther than max_panel_separation_m are not one gate;
 *  - bins: the role icon seen by the down camera hides the nearest bin
 *    without a role;
 *  - table and octagon: one xy for both, from table_octagon_primary (table,
 *    octagon or midpoint); the missing one is derived from the other (a
 *    table only with z_lock on, at floor_z - table_height_m).
 *
 * Landmarks made by a rule are marked `derived`.
 *
 * @param map The map after RetainedLandmarks::update().
 * @param config Rules.
 * @param vehicle_position Vehicle position in odom (the gate's front).
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

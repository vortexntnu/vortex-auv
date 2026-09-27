#ifndef LANDMARK_TARGETS__SLALOM_HPP_
#define LANDMARK_TARGETS__SLALOM_HPP_

#include <eigen3/Eigen/Dense>
#include <optional>
#include <vector>
#include <vortex/utils/types.hpp>
#include "landmark_targets/geometry.hpp"

namespace vortex::mission {

/**
 * @brief The three waypoints (odom x, y, yaw) that take the vehicle around the
 * slalom field back to the gate (AvoidSlalom), computed in the course frame:
 *  1. sideways out of the field: (reference.x, y_side)
 *  2. along the course past it: (return_x, y_side)
 *  3. sideways in, in front of the gate: (return_x, 0)
 * with the heading turned 180 deg (looking at the gate). y_side is midway
 * between the reference and the lane limit with most room.
 *
 * @param reference Where the last slalom layer was, in odom (x, y).
 * @param lane_y_min / lane_y_max Lane limits in course y (y to the right,
 * e.g. -6 and 6).
 * @param return_x Course x of the return point (in front of the gate).
 */
std::vector<vortex::utils::types::Pose> avoid_slalom_waypoints(
    const CourseFrame& course,
    const Eigen::Vector2d& reference,
    double lane_y_min,
    double lane_y_max,
    double return_x = 2.5,
    double z = 0.0);

}  // namespace vortex::mission

#endif  // LANDMARK_TARGETS__SLALOM_HPP_

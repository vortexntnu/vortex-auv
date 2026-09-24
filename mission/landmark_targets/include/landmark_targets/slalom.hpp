#ifndef LANDMARK_TARGETS__SLALOM_HPP_
#define LANDMARK_TARGETS__SLALOM_HPP_

#include <eigen3/Eigen/Dense>
#include <optional>
#include <vector>
#include <vortex/utils/types.hpp>
#include "landmark_targets/geometry.hpp"

namespace vortex::mission {

/// A slalom pipe from the map (odom position; pipes have no orientation).
struct Pipe {
    int id{-1};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

/// The gap between a red pipe and a white pipe.
struct PipeGap {
    /// Gap centre (odom x, y) and the heading to pass it (odom yaw).
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double heading{0.0};
    /// The red pipe the gap belongs to.
    int red_id{-1};
    /// gap - red pipe, for the next layer (the layers are aligned).
    Eigen::Vector2d offset_from_red{Eigen::Vector2d::Zero()};
};

/// What a passed layer tells the next: where the gap is relative to the red
/// pipe, and the heading used.
struct SlalomOffset {
    Eigen::Vector2d offset{Eigen::Vector2d::Zero()};
    double heading{0.0};
};

struct MatchPipesConfig {
    /// Only pipes at least this far in front of the vehicle count [m].
    double min_forward_m{1.0};
    /// A white pipe must be at least this far from the red one [m].
    double min_white_from_red_m{0.5};
    /// The red pipe must lie within this distance of the line between two
    /// white pipes [m], between @p min_projection and @p max_projection of it.
    double max_line_distance_m{0.5};
    double min_projection{0.1};
    double max_projection{0.9};
    /// The heading through the gap is turned this much towards the red pipe.
    double inward_deg{15.0};
};

/**
 * @brief Find the gap of the next slalom layer. ROS-free.
 *
 *  1. red pipes at least min_forward ahead of the vehicle and not passed;
 *  2. the nearest is the reference (none: nullopt);
 *  3. shortcut: with @p known_offset the target is the red pipe + offset;
 *  4. white pipes at least 0.5 m from the red one;
 *  5. best white pair with the red pipe on the line between them;
 *  6. left and right white pipe relative to the vehicle heading;
 *  7. gap = midpoint between the red pipe and the white pipe on the gate side,
 *     heading perpendicular to the line between the whites (closest to the
 *     current heading) turned inward;
 *  8. only one white pipe: right side -> midpoint, wrong side -> mirrored:
 *     red - (white - red) / 2.
 *
 * @param gate_side Which side of the vehicle the gap is on, as picked at the
 * gate (the gate panel that was chosen).
 * @param passed_red_ids Red pipes of layers that are behind the vehicle.
 */
std::optional<PipeGap> match_pipes(
    const std::vector<Pipe>& red,
    const std::vector<Pipe>& white,
    const vortex::utils::types::Pose& vehicle,
    Side gate_side,
    const std::vector<int>& passed_red_ids,
    const std::optional<SlalomOffset>& known_offset = std::nullopt,
    const MatchPipesConfig& config = {});

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

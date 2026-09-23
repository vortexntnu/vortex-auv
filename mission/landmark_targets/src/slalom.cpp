#include "landmark_targets/slalom.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vortex/utils/math.hpp>

namespace vortex::mission {

namespace {

using vortex::utils::math::ssa;

double yaw_of(const vortex::utils::types::Pose& pose) {
    const Eigen::Quaterniond q = pose.ori_quaternion().normalized();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// Turn @p heading by @p inward_rad towards the side where @p towards lies
/// (seen from @p from).
double turn_towards(double heading,
                    const Eigen::Vector2d& from,
                    const Eigen::Vector2d& towards,
                    double inward_rad) {
    const Eigen::Vector2d dir(std::cos(heading), std::sin(heading));
    const Eigen::Vector2d lateral = towards - from;
    const double cross = dir.x() * lateral.y() - dir.y() * lateral.x();
    if (std::abs(cross) < 1e-9) {
        return heading;
    }
    // NED: positive cross is clockwise, i.e. towards the right.
    return ssa(heading + (cross > 0.0 ? inward_rad : -inward_rad));
}

}  // namespace

std::optional<PipeGap> match_pipes(
    const std::vector<Pipe>& red,
    const std::vector<Pipe>& white,
    const vortex::utils::types::Pose& vehicle,
    Side gate_side,
    const std::vector<int>& passed_red_ids,
    const std::optional<SlalomOffset>& known_offset,
    const MatchPipesConfig& config) {
    // 1-2. The nearest red pipe in front that was not passed.
    const Pipe* reference = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const Pipe& r : red) {
        if (std::find(passed_red_ids.begin(), passed_red_ids.end(), r.id) !=
            passed_red_ids.end()) {
            continue;
        }
        if (forward_distance(vehicle, r.position) < config.min_forward_m) {
            continue;
        }
        const double d = (r.position - vehicle.pos_vector()).norm();
        if (d < best) {
            best = d;
            reference = &r;
        }
    }
    if (reference == nullptr) {
        return std::nullopt;
    }
    const Eigen::Vector2d red_xy = reference->position.head<2>();

    PipeGap gap;
    gap.red_id = reference->id;

    // 3. Shortcut: the layers are aligned, reuse the previous offset.
    if (known_offset) {
        gap.position = red_xy + known_offset->offset;
        gap.heading = known_offset->heading;
        gap.offset_from_red = known_offset->offset;
        return gap;
    }

    // 4. White pipes away from the red one.
    std::vector<const Pipe*> whites;
    for (const Pipe& w : white) {
        if ((w.position.head<2>() - red_xy).norm() >=
            config.min_white_from_red_m) {
            whites.push_back(&w);
        }
    }
    if (whites.empty()) {
        return std::nullopt;
    }

    // 5. The best pair with the red pipe on the line between them.
    const Pipe* left = nullptr;
    const Pipe* right = nullptr;
    double best_line = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < whites.size(); ++i) {
        for (std::size_t j = i + 1; j < whites.size(); ++j) {
            const Eigen::Vector2d a = whites[i]->position.head<2>();
            const Eigen::Vector2d b = whites[j]->position.head<2>();
            const Eigen::Vector2d ab = b - a;
            const double len2 = ab.squaredNorm();
            if (len2 < 1e-6) {
                continue;
            }
            const double t = (red_xy - a).dot(ab) / len2;
            if (t < config.min_projection || t > config.max_projection) {
                continue;
            }
            const double line_dist = (red_xy - (a + t * ab)).norm();
            if (line_dist > config.max_line_distance_m ||
                line_dist >= best_line) {
                continue;
            }
            best_line = line_dist;
            // 6. Left and right relative to the vehicle heading.
            const bool i_is_left =
                side_of(vehicle, whites[i]->position) == Side::LEFT;
            const bool j_is_left =
                side_of(vehicle, whites[j]->position) == Side::LEFT;
            if (i_is_left == j_is_left) {
                // Both on one side: order by lateral position instead.
                const double yaw = yaw_of(vehicle);
                const Eigen::Vector2d left_dir(std::sin(yaw), -std::cos(yaw));
                const bool i_more_left =
                    (a - vehicle.pos_vector().head<2>()).dot(left_dir) >
                    (b - vehicle.pos_vector().head<2>()).dot(left_dir);
                left = i_more_left ? whites[i] : whites[j];
                right = i_more_left ? whites[j] : whites[i];
            } else {
                left = i_is_left ? whites[i] : whites[j];
                right = i_is_left ? whites[j] : whites[i];
            }
        }
    }

    const double inward = config.inward_deg * M_PI / 180.0;
    const double yaw = yaw_of(vehicle);

    if (left != nullptr && right != nullptr) {
        // 7. Gap between the red pipe and the white pipe on the gate side.
        const Pipe* chosen = gate_side == Side::LEFT ? left : right;
        gap.position = 0.5 * (red_xy + chosen->position.head<2>());
        const double perpendicular = perpendicular_heading(
            left->position.head<2>(), right->position.head<2>(), yaw);
        gap.heading = turn_towards(perpendicular, gap.position, red_xy, inward);
        gap.offset_from_red = gap.position - red_xy;
        return gap;
    }

    // 8. One white pipe.
    const Pipe* nearest = *std::min_element(
        whites.begin(), whites.end(), [&](const Pipe* a, const Pipe* b) {
            return (a->position.head<2>() - red_xy).norm() <
                   (b->position.head<2>() - red_xy).norm();
        });
    const Eigen::Vector2d white_xy = nearest->position.head<2>();
    const bool right_side = side_of(vehicle, nearest->position) == gate_side;
    if (right_side) {
        gap.position = Eigen::Vector2d(0.5 * (red_xy + white_xy));
    } else {
        gap.position = Eigen::Vector2d(red_xy - 0.5 * (white_xy - red_xy));
    }
    const double perpendicular = perpendicular_heading(red_xy, white_xy, yaw);
    gap.heading = turn_towards(perpendicular, gap.position, red_xy, inward);
    gap.offset_from_red = gap.position - red_xy;
    return gap;
}

std::vector<vortex::utils::types::Pose> avoid_slalom_waypoints(
    const CourseFrame& course,
    const Eigen::Vector2d& reference,
    double lane_left_m,
    double lane_right_m,
    double return_x,
    double z) {
    const Eigen::Vector2d ref = to_course(course, reference);

    // Room on each side of the reference (left limit positive, right limit
    // negative).
    const double room_left = lane_left_m - ref.y();
    const double room_right = ref.y() - lane_right_m;
    const double y_side = room_left >= room_right
                              ? 0.5 * (ref.y() + lane_left_m)
                              : 0.5 * (ref.y() + lane_right_m);

    const double heading = ssa(course.through_yaw + M_PI);
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    const auto pose_at = [&](double cx, double cy) {
        const Eigen::Vector2d odom = from_course(course, {cx, cy});
        return vortex::utils::types::Pose::from_eigen(
            Eigen::Vector3d(odom.x(), odom.y(), z), q);
    };
    return {pose_at(ref.x(), y_side), pose_at(return_x, y_side),
            pose_at(return_x, 0.0)};
}

}  // namespace vortex::mission

#include "landmark_targets/slalom.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::mission {

using vortex::utils::math::ssa;

std::vector<vortex::utils::types::Pose> avoid_slalom_waypoints(
    const CourseFrame& course,
    const Eigen::Vector2d& reference,
    double lane_y_min,
    double lane_y_max,
    double return_x,
    double z) {
    const Eigen::Vector2d ref = to_course(course, reference);

    // Go out on the side of the reference with the most room to the lane
    // limit, halfway to it.
    const double room_min = ref.y() - lane_y_min;
    const double room_max = lane_y_max - ref.y();
    const double y_side = room_max >= room_min ? 0.5 * (ref.y() + lane_y_max)
                                               : 0.5 * (ref.y() + lane_y_min);

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

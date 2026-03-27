#include "reference_filter_dp_quat/lib/waypoint_utils.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::guidance {

Pose compute_waypoint_goal(const Pose& incoming_waypoint,
                           WaypointMode mode,
                           const Pose& current_state) {
    Pose waypoint_out = incoming_waypoint;

    switch (mode) {
        case WaypointMode::FULL_POSE:
            break;

        case WaypointMode::ONLY_POSITION:
            waypoint_out.set_ori(current_state.ori_quaternion());
            break;

        case WaypointMode::FORWARD_HEADING: {
            double dx = incoming_waypoint.x - current_state.x;
            double dy = incoming_waypoint.y - current_state.y;
            double forward_heading = std::atan2(dy, dx);

            waypoint_out.set_ori(Eigen::Quaterniond(
                Eigen::AngleAxisd(forward_heading, Eigen::Vector3d::UnitZ())));
            break;
        }

        case WaypointMode::ONLY_ORIENTATION:
            waypoint_out.set_pos(current_state.pos_vector());
            break;
    }

    return waypoint_out;
}

bool has_converged(const Pose& state,
                   const Pose& waypoint_goal,
                   WaypointMode mode,
                   double convergence_threshold) {
    const Eigen::Vector3d ep = state.pos_vector() - waypoint_goal.pos_vector();

    const Eigen::Vector3d ea = vortex::utils::math::quaternion_error(
        state.ori_quaternion(), waypoint_goal.ori_quaternion());

    const double err = [&] {
        switch (mode) {
            case WaypointMode::ONLY_POSITION:
                return ep.norm();
            case WaypointMode::ONLY_ORIENTATION:
                return ea.norm();
            case WaypointMode::FORWARD_HEADING:
                return std::sqrt(ep.squaredNorm() + ea(2) * ea(2));
            case WaypointMode::FULL_POSE:
            default:
                return std::sqrt(ep.squaredNorm() + ea.squaredNorm());
        }
    }();

    return err < convergence_threshold;
}

}  // namespace vortex::guidance

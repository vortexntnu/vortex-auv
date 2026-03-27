#include "reference_filter_dp/lib/waypoint_utils.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include <vortex/utils/waypoint_utils.hpp>

namespace vortex::guidance {

namespace {
using vortex::utils::types::PoseEuler;
using vortex::utils::types::Pose;
        
Pose vec_to_pose(const Eigen::Vector6d& v) {
    return PoseEuler{.x = v(0), .y = v(1), .z = v(2),
                     .roll = v(3), .pitch = v(4), .yaw = v(5)}
        .as_pose();
}
}  // namespace

Eigen::Vector6d apply_mode_logic(const Eigen::Vector6d& reference_in,
                                 WaypointMode mode,
                                 const Eigen::Vector6d& current_state) {
    Eigen::Vector6d reference_out = reference_in;

    using vortex::utils::math::ssa;

    switch (mode) {
        case WaypointMode::FULL_POSE:
            reference_out(3) =
                current_state(3) + ssa(reference_in(3) - current_state(3));
            reference_out(4) =
                current_state(4) + ssa(reference_in(4) - current_state(4));
            reference_out(5) =
                current_state(5) + ssa(reference_in(5) - current_state(5));
            break;

        case WaypointMode::ONLY_POSITION:
        case WaypointMode::FORWARD_HEADING:
        case WaypointMode::ONLY_ORIENTATION:
            reference_out = vortex::utils::waypoints::compute_waypoint_goal(
                                vec_to_pose(reference_in), mode,
                                vec_to_pose(current_state))
                                .as_pose_euler()
                                .to_vector();
            break;
    }

    return reference_out;
}

bool has_converged(const Eigen::Vector6d& measured_pose,
                   const Eigen::Vector6d& reference,
                   WaypointMode mode,
                   double convergence_threshold) {
    return vortex::utils::waypoints::has_converged(
        vec_to_pose(measured_pose), vec_to_pose(reference), mode,
        convergence_threshold);
}

}  // namespace vortex::guidance

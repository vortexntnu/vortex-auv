#include "reference_filter_dp/lib/waypoint_utils.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::guidance {

Eigen::Vector6d apply_mode_logic(const Eigen::Vector6d& r_in,
                                 WaypointMode mode,
                                 const Eigen::Vector6d& current_state) {
    Eigen::Vector6d r_out = r_in;

    switch (mode) {
        case WaypointMode::FULL_POSE:
            break;

        case WaypointMode::ONLY_POSITION:
            r_out(3) = current_state(3);
            r_out(4) = current_state(4);
            r_out(5) = current_state(5);
            break;

        case WaypointMode::FORWARD_HEADING: {
            double dx = r_in(0) - current_state(0);
            double dy = r_in(1) - current_state(1);
            double forward_heading = std::atan2(dy, dx);

            r_out(3) = 0.0;
            r_out(4) = 0.0;
            r_out(5) = vortex::utils::math::ssa(forward_heading);
            break;
        }

        case WaypointMode::ONLY_ORIENTATION:
            r_out(0) = current_state(0);
            r_out(1) = current_state(1);
            r_out(2) = current_state(2);
            break;
    }

    return r_out;
}

bool has_converged(const Eigen::Vector6d& measured_pose,
                   const Eigen::Vector6d& reference,
                   WaypointMode mode,
                   double convergence_threshold) {
    const Eigen::Vector3d ep = measured_pose.head<3>() - reference.head<3>();

    Eigen::Vector3d ea;
    ea(0) = vortex::utils::math::ssa(measured_pose(3) - reference(3));
    ea(1) = vortex::utils::math::ssa(measured_pose(4) - reference(4));
    ea(2) = vortex::utils::math::ssa(measured_pose(5) - reference(5));

    double err = 0.0;

    switch (mode) {
        case WaypointMode::ONLY_POSITION:
            err = ep.norm();
            break;

        case WaypointMode::ONLY_ORIENTATION:
            err = ea.norm();
            break;

        case WaypointMode::FORWARD_HEADING:
            err = std::sqrt(ep.squaredNorm() + ea(2) * ea(2));
            break;

        case WaypointMode::FULL_POSE:
        default:
            err = std::sqrt(ep.squaredNorm() + ea.squaredNorm());
            break;
    }

    return err < convergence_threshold;
}

}  // namespace vortex::guidance

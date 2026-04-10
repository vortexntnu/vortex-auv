#include "reference_filter_dp/lib/waypoint_utils.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::guidance {

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
            reference_out(3) = current_state(3);
            reference_out(4) = current_state(4);
            reference_out(5) = current_state(5);
            break;

        case WaypointMode::FORWARD_HEADING: {
            double dx = reference_in(0) - current_state(0);
            double dy = reference_in(1) - current_state(1);
            double forward_heading = std::atan2(dy, dx);

            reference_out(3) = 0.0;
            reference_out(4) = 0.0;
            reference_out(5) = vortex::utils::math::ssa(forward_heading);
            break;
        }

        case WaypointMode::ONLY_ORIENTATION:
            reference_out(0) = current_state(0);
            reference_out(1) = current_state(1);
            reference_out(2) = current_state(2);
            reference_out(3) =
                current_state(3) + ssa(reference_in(3) - current_state(3));
            reference_out(4) =
                current_state(4) + ssa(reference_in(4) - current_state(4));
            reference_out(5) =
                current_state(5) + ssa(reference_in(5) - current_state(5));
            break;
    }

    return reference_out;
}

bool has_converged(const Eigen::Vector6d& measured_pose,
                   const Eigen::Vector6d& reference,
                   WaypointMode mode,
                   double convergence_threshold) {
    using vortex::utils::math::ssa;
    const Eigen::Vector3d ep = measured_pose.head<3>() - reference.head<3>();

    Eigen::Vector3d ea;
    ea(0) = ssa(measured_pose(3) - reference(3));
    ea(1) = ssa(measured_pose(4) - reference(4));
    ea(2) = ssa(measured_pose(5) - reference(5));

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

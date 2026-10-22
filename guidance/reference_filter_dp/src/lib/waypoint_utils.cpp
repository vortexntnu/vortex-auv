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

        case WaypointMode::POSITION_AND_YAW:
            reference_out(3) = 0.0;
            reference_out(4) = 0.0;
            reference_out(5) =
                current_state(5) + ssa(reference_in(5) - current_state(5));
            break;

        case WaypointMode::XY_AND_YAW:
            reference_out(2) = current_state(2);
            reference_out(3) = 0.0;
            reference_out(4) = 0.0;
            reference_out(5) =
                current_state(5) + ssa(reference_in(5) - current_state(5));
            break;

        case WaypointMode::ONLY_Z:
            reference_out(0) = current_state(0);
            reference_out(1) = current_state(1);
            reference_out(3) = current_state(3);
            reference_out(4) = current_state(4);
            reference_out(5) = current_state(5);
            break;
    }

    return reference_out;
}

bool has_converged(const Eigen::Vector6d& measured_pose,
                   const Eigen::Vector6d& reference,
                   WaypointMode mode,
                   const ConvergenceTolerance& tolerance) {
    using vortex::utils::math::ssa;
    const Eigen::Vector3d position_error =
        measured_pose.head<3>() - reference.head<3>();

    Eigen::Vector3d orientation_error;
    orientation_error(0) = ssa(measured_pose(3) - reference(3));
    orientation_error(1) = ssa(measured_pose(4) - reference(4));
    orientation_error(2) = ssa(measured_pose(5) - reference(5));

    const ControlledError err = [&]() -> ControlledError {
        switch (mode) {
            case WaypointMode::ONLY_POSITION:
                return {position_error.norm(), 0.0};
            case WaypointMode::ONLY_ORIENTATION:
                return {0.0, orientation_error.norm()};
            case WaypointMode::FORWARD_HEADING:
            case WaypointMode::POSITION_AND_YAW:
                return {position_error.norm(), std::abs(orientation_error(2))};
            case WaypointMode::XY_AND_YAW:
                return {position_error.head<2>().norm(),
                        std::abs(orientation_error(2))};
            case WaypointMode::ONLY_Z:
                return {std::abs(position_error(2)), 0.0};
            case WaypointMode::FULL_POSE:
            default:
                return {position_error.norm(), orientation_error.norm()};
        }
    }();

    const bool position_ok =
        tolerance.position <= 0.0 || err.position < tolerance.position;
    const bool orientation_ok =
        tolerance.orientation <= 0.0 || err.orientation < tolerance.orientation;
    return position_ok && orientation_ok;
}

}  // namespace vortex::guidance

#include "reference_filter_dp/lib/waypoint_follower.hpp"
#include "reference_filter_dp/lib/waypoint_utils.hpp"

namespace vortex::guidance {

WaypointFollower::WaypointFollower(const ReferenceFilterParams& params,
                                   double dt_seconds)
    : filter_(params), dt_seconds_(dt_seconds) {}

void WaypointFollower::start(const Eigen::Vector18d& initial_state,
                             const Waypoint& waypoint,
                             double convergence_threshold) {
    x_ = initial_state;
    waypoint_ = waypoint;
    convergence_threshold_ = convergence_threshold;
    r_ = apply_mode_logic(waypoint.pose, waypoint.mode, x_.head<6>());
}

StepResult WaypointFollower::step(const Eigen::Vector6d& measured_pose) {
    Eigen::Vector18d x_dot = filter_.calculate_x_dot(x_, r_);
    x_ += x_dot * dt_seconds_;

    bool converged = has_converged(measured_pose, r_, waypoint_.mode,
                                   convergence_threshold_);

    return StepResult{x_, r_, converged};
}

void WaypointFollower::set_reference(const Eigen::Vector6d& r,
                                     WaypointMode mode) {
    r_ = apply_mode_logic(r, mode, x_.head<6>());
}

const Eigen::Vector18d& WaypointFollower::state() const {
    return x_;
}

const Eigen::Vector6d& WaypointFollower::reference() const {
    return r_;
}

void WaypointFollower::snap_state_to_reference() {
    x_.head<6>() = r_;
}

}  // namespace vortex::guidance

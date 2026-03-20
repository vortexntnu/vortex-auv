#include "reference_filter_dp/lib/waypoint_follower.hpp"
#include <vortex/utils/math.hpp>
#include "reference_filter_dp/lib/waypoint_utils.hpp"

namespace vortex::guidance {

WaypointFollower::WaypointFollower(const ReferenceFilterParams& params,
                                   double dt_seconds)
    : filter_(params), dt_seconds_(dt_seconds) {}

void WaypointFollower::start(const PoseEuler& pose,
                             const Twist& twist,
                             const Waypoint& waypoint,
                             double convergence_threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = compute_initial_state(pose, twist);
    waypoint_ = waypoint.mode;
    convergence_threshold_ = convergence_threshold;
    reference_goal_ = apply_mode_logic(waypoint.pose.to_vector(), waypoint.mode,
                                       x_.head<6>());
}

Eigen::Vector18d WaypointFollower::compute_initial_state(const PoseEuler& pose,
                                                         const Twist& twist) {
    Eigen::Vector18d x = Eigen::Vector18d::Zero();

    Eigen::Vector6d pose_vec = pose.to_vector();
    pose_vec(3) = vortex::utils::math::ssa(pose_vec(3));
    pose_vec(4) = vortex::utils::math::ssa(pose_vec(4));
    pose_vec(5) = vortex::utils::math::ssa(pose_vec(5));
    x.head<6>() = pose_vec;

    Eigen::Matrix<double, 6, 6> J = pose.as_j_matrix();
    x.segment<6>(6) = J * twist.to_vector();

    return x;
}

StepResult WaypointFollower::step(const Eigen::Vector6d& measured_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Vector18d x_dot = filter_.calculate_x_dot(x_, reference_goal_);
    x_ += x_dot * dt_seconds_;

    bool converged = has_converged(measured_pose, reference_goal_, waypoint_,
                                   convergence_threshold_);

    return StepResult{x_, reference_goal_, converged};
}

void WaypointFollower::set_reference(const PoseEuler& reference_goal_pose,
                                     WaypointMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    reference_goal_ =
        apply_mode_logic(reference_goal_pose.to_vector(), mode, x_.head<6>());
}

Eigen::Vector18d WaypointFollower::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return x_;
}

Eigen::Vector6d WaypointFollower::reference() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return reference_goal_;
}

void WaypointFollower::snap_state_to_reference() {
    std::lock_guard<std::mutex> lock(mutex_);
    x_.head<6>() = reference_goal_;
}

}  // namespace vortex::guidance

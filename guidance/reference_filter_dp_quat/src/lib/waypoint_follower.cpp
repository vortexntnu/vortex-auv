#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"
#include <vortex/utils/math.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"
#include "reference_filter_dp_quat/lib/waypoint_utils.hpp"

namespace vortex::guidance {

WaypointFollower::WaypointFollower(const ReferenceFilterParams& params,
                                   double dt_seconds)
    : filter_(params), dt_seconds_(dt_seconds) {}

void WaypointFollower::start(const PoseEuler& pose,
                             const Twist& twist,
                             const Waypoint& waypoint,
                             double convergence_threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = compute_initial_state(pose, twist);
    waypoint_mode_ = waypoint.mode;
    convergence_threshold_ = convergence_threshold;
    reference_goal_ = apply_mode_logic(waypoint.pose.to_vector(),
                                       waypoint_mode_, state_.head<6>());
}

Eigen::Vector18d WaypointFollower::compute_initial_state(const PoseEuler& pose,
                                                         const Twist& twist) {
    Eigen::Vector18d x = Eigen::Vector18d::Zero();

    x.head<6>() = pose.to_vector();

    Eigen::Matrix<double, 6, 6> J = pose.as_j_matrix();
    x.segment<6>(6) = J * twist.to_vector();

    return x;
}

Eigen::Vector18d WaypointFollower::step() {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Vector18d state_dot_ =
        filter_.calculate_x_dot(state_, reference_goal_);
    state_ += state_dot_ * dt_seconds_;

    return state_;
}

bool WaypointFollower::within_convergance(
    const Eigen::Vector6d& measured_pose) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_converged(measured_pose, reference_goal_, waypoint_mode_,
                         convergence_threshold_);
}

void WaypointFollower::set_reference(const PoseEuler& reference_goal_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    reference_goal_ = apply_mode_logic(reference_goal_pose.to_vector(),
                                       waypoint_mode_, state_.head<6>());
}

Eigen::Vector18d WaypointFollower::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

Eigen::Vector6d WaypointFollower::reference() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return reference_goal_;
}

void WaypointFollower::snap_state_to_reference() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.head<6>() = reference_goal_;
}

}  // namespace vortex::guidance

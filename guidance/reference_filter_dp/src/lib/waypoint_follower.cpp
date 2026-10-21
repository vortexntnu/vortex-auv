#include "reference_filter_dp/lib/waypoint_follower.hpp"
#include <vortex/utils/math.hpp>
#include "reference_filter_dp/lib/eigen_typedefs.hpp"
#include "reference_filter_dp/lib/waypoint_utils.hpp"

namespace vortex::guidance {

WaypointFollower::WaypointFollower(const ReferenceFilterParams& params,
                                   double dt_seconds)
    : filter_(params), dt_seconds_(dt_seconds) {}

void WaypointFollower::start(const PoseEuler& pose,
                             const Twist& twist,
                             const Waypoint& waypoint,
                             const ConvergenceTolerance& tolerance) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = compute_initial_state(pose, twist);
    waypoint_mode_ = waypoint.mode;
    tolerance_ = tolerance;
    inside_since_sec_.reset();
    reference_goal_ = apply_mode_logic(waypoint.pose.to_vector(),
                                       waypoint_mode_, state_.head<6>());
}

void WaypointFollower::retarget(const Waypoint& waypoint,
                                const ConvergenceTolerance& tolerance) {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoint_mode_ = waypoint.mode;
    tolerance_ = tolerance;
    inside_since_sec_.reset();
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

bool WaypointFollower::within_locked(
    const Eigen::Vector6d& measured_pose) const {
    return has_converged(measured_pose, reference_goal_, waypoint_mode_,
                         tolerance_);
}

bool WaypointFollower::within_convergance(
    const Eigen::Vector6d& measured_pose) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return within_locked(measured_pose);
}

bool WaypointFollower::update_convergence(const Eigen::Vector6d& measured_pose,
                                          double t_sec,
                                          double hold_time_sec,
                                          bool ignore_z) {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Vector6d pose = measured_pose;
    if (ignore_z) {
        pose(2) = reference_goal_(2);
    }
    if (!within_locked(pose)) {
        inside_since_sec_.reset();
        return false;
    }
    if (!inside_since_sec_) {
        inside_since_sec_ = t_sec;
    }
    return t_sec - *inside_since_sec_ >= hold_time_sec;
}

void WaypointFollower::update_z_goal(double target_ned_z) {
    std::lock_guard<std::mutex> lock(mutex_);
    reference_goal_(2) = target_ned_z;
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

#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"
#include <vortex/utils/math.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"
#include "reference_filter_dp_quat/lib/waypoint_utils.hpp"

namespace vortex::guidance {

WaypointFollower::WaypointFollower(const ReferenceFilterParams& params,
                                   double dt_seconds)
    : filter_(params), dt_seconds_(dt_seconds) {}

void WaypointFollower::start(const Pose& pose,
                             const Twist& twist,
                             const Waypoint& waypoint,
                             double convergence_threshold) {
    std::lock_guard<std::mutex> lock(mutex_);

    nominal_pose_ = pose;
    state_ = Eigen::Vector18d::Zero();

    Eigen::Matrix3d R = pose.as_rotation_matrix();
    Eigen::Vector6d twist_vec = twist.to_vector();
    state_.segment<3>(6) = R * twist_vec.head<3>();
    state_.segment<3>(9) = R * twist_vec.tail<3>();

    waypoint_mode_ = waypoint.mode;
    convergence_threshold_ = convergence_threshold;
    waypoint_goal_ =
        compute_waypoint_goal(waypoint.pose, waypoint_mode_, nominal_pose_);
}

void WaypointFollower::step() {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector6d r;
    r.head<3>() = waypoint_goal_.pos_vector() - nominal_pose_.pos_vector();
    r.tail<3>() = vortex::utils::math::quaternion_error(
        nominal_pose_.ori_quaternion(), waypoint_goal_.ori_quaternion());

    Eigen::Vector18d x_dot = filter_.calculate_x_dot(state_, r);
    state_ += x_dot * dt_seconds_;

    nominal_pose_.set_pos(nominal_pose_.pos_vector() + state_.head<3>());
    state_.head<3>().setZero();

    Eigen::Vector3d dphi = state_.segment<3>(3);
    double angle = dphi.norm();
    if (angle >= 1e-10) {
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dphi.normalized()));
        nominal_pose_.set_ori(nominal_pose_.ori_quaternion() * dq);
        state_.segment<3>(3).setZero();
    }
}

bool WaypointFollower::within_convergance(const Pose& measured_pose) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_converged(measured_pose, waypoint_goal_, waypoint_mode_,
                         convergence_threshold_);
}

void WaypointFollower::set_reference(const Pose& reference_goal_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoint_goal_ = compute_waypoint_goal(reference_goal_pose, waypoint_mode_,
                                           nominal_pose_);
}

void WaypointFollower::snap_state_to_reference() {
    std::lock_guard<std::mutex> lock(mutex_);
    nominal_pose_ = waypoint_goal_;
    state_.head<12>().setZero();
}

Pose WaypointFollower::pose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return nominal_pose_;
}

Eigen::Vector6d WaypointFollower::velocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.segment<6>(6);
}

Pose WaypointFollower::waypoint_goal() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return waypoint_goal_;
}

}  // namespace vortex::guidance

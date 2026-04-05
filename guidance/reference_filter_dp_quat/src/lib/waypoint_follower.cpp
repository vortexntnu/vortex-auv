#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"
#include <vortex/utils/math.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"
#include <vortex/utils/types.hpp>

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
        vortex::utils::waypoints::compute_waypoint_goal(waypoint.pose, waypoint_mode_, nominal_pose_);
}

void WaypointFollower::step() {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Vector6d filter_reference = update_reference();

    const Eigen::Vector18d state_derivative =
        filter_.calculate_x_dot(state_, filter_reference);
    state_ += state_derivative * dt_seconds_;
    inject_and_reset();
}

Eigen::Vector6d WaypointFollower::update_reference() const {
    Eigen::Vector6d filter_reference;
    filter_reference.head<3>() =
        waypoint_goal_.pos_vector() - nominal_pose_.pos_vector();
    filter_reference.tail<3>() = vortex::utils::math::quaternion_error(
        nominal_pose_.ori_quaternion(), waypoint_goal_.ori_quaternion());
    return filter_reference;
}

void WaypointFollower::inject_and_reset() {
    nominal_pose_.set_pos(nominal_pose_.pos_vector() + state_.head<3>());
    state_.head<3>().setZero();

    const Eigen::Vector3d delta_orientation = state_.segment<3>(3);
    const double angle = delta_orientation.norm();
    if (angle >= 1e-10) {
        Eigen::Quaterniond delta_quat(
            Eigen::AngleAxisd(angle, delta_orientation.normalized()));
        Eigen::Quaterniond q_new =
            nominal_pose_.ori_quaternion() * delta_quat;
        // Enforce positive hemisphere to prevent sign flips in the published
        // reference quaternion that would cause the downstream controller to
        // see large spurious orientation errors.
        if (q_new.w() < 0.0) {
            q_new.coeffs() = -q_new.coeffs();
        }
        nominal_pose_.set_ori(q_new);
        state_.segment<3>(3).setZero();
    }
}

bool WaypointFollower::within_convergance(const Pose& measured_pose) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vortex::utils::waypoints::has_converged(measured_pose, waypoint_goal_, waypoint_mode_,
                         convergence_threshold_);
}

void WaypointFollower::set_reference(const Pose& reference_goal_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoint_goal_ = vortex::utils::waypoints::compute_waypoint_goal(reference_goal_pose, waypoint_mode_,
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

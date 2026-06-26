#include "reference_filter_dp_quat/lib/waypoint_guidance_manager.hpp"

#include <stdexcept>

namespace vortex::guidance {

WaypointGuidanceManager::WaypointGuidanceManager(
    const WaypointGuidanceManagerConfig& config)
    : config_(config)
{
    if (config_.time_step.count() <= 0) {
        throw std::invalid_argument(
            "WaypointGuidanceManager time_step must be greater than zero");
    }

    if (config_.altitude_low_pass_alpha < 0.0 ||
        config_.altitude_low_pass_alpha > 1.0) {
        throw std::invalid_argument(
            "altitude_low_pass_alpha must be in the range [0.0, 1.0]");
    }

    recreate_follower();
}

void WaypointGuidanceManager::set_pose(
    const vortex::utils::types::Pose& pose)
{
    current_pose_ = pose;
}

void WaypointGuidanceManager::set_twist(
    const vortex::utils::types::Twist& twist)
{
    current_twist_ = twist;
}

void WaypointGuidanceManager::set_altitude(double altitude_m)
{
    if (altitude_m <= 0.0) {
        return;
    }

    if (!altitude_valid_) {
        current_altitude_ = altitude_m;
        altitude_valid_ = true;
        return;
    }

    current_altitude_ =
        config_.altitude_low_pass_alpha * current_altitude_ +
        (1.0 - config_.altitude_low_pass_alpha) * altitude_m;
}

WaypointStatus WaypointGuidanceManager::submit_waypoint(
    vortex::utils::types::Waypoint waypoint,
    double convergence_threshold)
{
    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
    }

    if (!prepare_altitude_goal(waypoint)) {
        waypoint_active_ = false;
        status_ = WaypointStatus::rejected;
        return status_;
    }

    if (waypoint_active_) {
        follower_->retarget(waypoint, convergence_threshold);
    } else {
        follower_->start(
            current_pose_,
            current_twist_,
            waypoint,
            convergence_threshold);
    }

    waypoint_active_ = true;

    keep_altitude_ =
        waypoint.keep_altitude && config_.altitude_control_enabled;

    require_altitude_convergence_ =
        waypoint.require_altitude_convergence;

    desired_altitude_ = waypoint.desired_altitude;
    convergence_threshold_ = convergence_threshold;

    status_ = WaypointStatus::running;

    return status_;
}

void WaypointGuidanceManager::cancel_waypoint()
{
    waypoint_active_ = false;

    keep_altitude_ = false;
    require_altitude_convergence_ = true;

    desired_altitude_ = 0.0;
    convergence_threshold_ = 0.1;

    status_ = WaypointStatus::canceled;
}

void WaypointGuidanceManager::reset()
{
    waypoint_active_ = false;

    keep_altitude_ = false;
    require_altitude_convergence_ = true;

    desired_altitude_ = 0.0;
    convergence_threshold_ = 0.1;

    recreate_follower();

    status_ = WaypointStatus::idle;
}

GuidanceReference WaypointGuidanceManager::tick()
{
    GuidanceReference output{
        .pose = current_pose_,
        .twist = vortex::utils::types::Twist{},
        .status = status_,
        .active = waypoint_active_,
        .just_completed = false,
    };

    if (!waypoint_active_) {
        return output;
    }

    follower_->step();

    if (keep_altitude_ && altitude_valid_) {
        const double z_goal =
            current_pose_.z + current_altitude_ - desired_altitude_;

        follower_->update_z_goal(z_goal);
    }

    output.pose = follower_->pose();
    output.twist = follower_->velocity();
    output.status = status_;
    output.active = true;

    const bool converged =
        (keep_altitude_ && !require_altitude_convergence_)
            ? follower_->within_convergance_ignore_z(current_pose_)
            : follower_->within_convergance(current_pose_);

    if (!converged) {
        return output;
    }

    follower_->snap_state_to_reference();

    waypoint_active_ = false;
    keep_altitude_ = false;

    status_ = WaypointStatus::succeeded;

    output.pose = follower_->pose();
    output.twist = follower_->velocity();
    output.status = status_;
    output.active = false;
    output.just_completed = true;

    return output;
}

WaypointStatus WaypointGuidanceManager::status() const noexcept
{
    return status_;
}

bool WaypointGuidanceManager::active() const noexcept
{
    return waypoint_active_;
}

bool WaypointGuidanceManager::altitude_valid() const noexcept
{
    return altitude_valid_;
}

double WaypointGuidanceManager::altitude() const noexcept
{
    return current_altitude_;
}

const vortex::utils::types::Pose&
WaypointGuidanceManager::current_goal() const
{
    return follower_->waypoint_goal();
}

bool WaypointGuidanceManager::prepare_altitude_goal(
    vortex::utils::types::Waypoint& waypoint)
{
    if (!waypoint.keep_altitude) {
        return true;
    }

    if (!config_.altitude_control_enabled) {
        // Preserve the old behavior: warn in the caller if desired, but
        // continue using the waypoint z coordinate as supplied.
        return true;
    }

    if (waypoint.desired_altitude <= 0.0) {
        return false;
    }

    if (!altitude_valid_) {
        // Preserve old behavior: allow the waypoint to start using its
        // supplied z coordinate until valid altitude arrives.
        return true;
    }

    waypoint.pose.z =
        current_pose_.z + current_altitude_ - waypoint.desired_altitude;

    return true;
}

void WaypointGuidanceManager::recreate_follower()
{
    const double time_step_s =
        static_cast<double>(config_.time_step.count()) / 1000.0;

    follower_ = std::make_unique<WaypointFollower>(
        config_.filter_params,
        time_step_s);
}

}  // namespace vortex::guidance

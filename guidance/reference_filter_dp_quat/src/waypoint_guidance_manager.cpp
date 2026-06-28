#include "reference_filter_dp_quat/waypoint_guidance_manager.hpp"

#include <stdexcept>

namespace {

vortex::utils::types::Twist vector_to_twist(const Eigen::Vector6d& velocity) {
    return {
        .u = velocity(0),
        .v = velocity(1),
        .w = velocity(2),
        .p = velocity(3),
        .q = velocity(4),
        .r = velocity(5),
    };
}

}  // namespace

namespace vortex::guidance {

WaypointGuidanceManager::WaypointGuidanceManager(
    const WaypointGuidanceManagerConfig& config)
    : config_(config) {
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

WaypointStatus WaypointGuidanceManager::submit_waypoint(
    vortex::utils::types::Waypoint waypoint,
    const vortex::utils::types::Pose& pose,
    const vortex::utils::types::Twist& twist,
    double convergence_threshold) {
    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
    }

    if (!prepare_altitude_goal(waypoint, pose)) {
        waypoint_active_ = false;
        status_ = WaypointStatus::rejected;
        return status_;
    }

    if (waypoint_active_) {
        follower_->retarget(waypoint, convergence_threshold);
    } else {
        follower_->start(pose, twist, waypoint, convergence_threshold);
    }

    waypoint_active_ = true;
    keep_altitude_ = waypoint.keep_altitude && config_.altitude_control_enabled;
    require_altitude_convergence_ = waypoint.require_altitude_convergence;
    desired_altitude_ = waypoint.desired_altitude;
    convergence_threshold_ = convergence_threshold;
    status_ = WaypointStatus::running;

    return status_;
}

void WaypointGuidanceManager::cancel_waypoint() {
    waypoint_active_ = false;

    keep_altitude_ = false;
    require_altitude_convergence_ = true;

    desired_altitude_ = 0.0;
    convergence_threshold_ = 0.1;

    status_ = WaypointStatus::canceled;
}

void WaypointGuidanceManager::reset() {
    waypoint_active_ = false;

    keep_altitude_ = false;
    require_altitude_convergence_ = true;

    desired_altitude_ = 0.0;
    convergence_threshold_ = 0.1;

    recreate_follower();

    status_ = WaypointStatus::idle;
}

GuidanceReference WaypointGuidanceManager::tick(
    const vortex::utils::types::Pose& pose,
    std::optional<double> altitude_m) {
    if (altitude_m) {
        update_altitude(*altitude_m);
    }

    GuidanceReference output{
        .pose = pose,
        .twist = {},
        .status = status_,
        .active = waypoint_active_,
        .just_completed = false,
    };

    if (!waypoint_active_) {
        return output;
    }

    follower_->step();

    if (keep_altitude_ && altitude_valid_) {
        const double z_goal = pose.z + current_altitude_ - desired_altitude_;

        follower_->update_z_goal(z_goal);
    }

    output.pose = follower_->pose();
    output.twist = vector_to_twist(follower_->velocity());
    output.status = status_;
    output.active = true;

    const bool converged = (keep_altitude_ && !require_altitude_convergence_)
                               ? follower_->within_convergance_ignore_z(pose)
                               : follower_->within_convergance(pose);

    if (!converged) {
        return output;
    }

    follower_->snap_state_to_reference();

    waypoint_active_ = false;
    keep_altitude_ = false;
    status_ = WaypointStatus::succeeded;

    output.pose = follower_->pose();
    output.twist = vector_to_twist(follower_->velocity());
    output.status = status_;
    output.active = false;
    output.just_completed = true;

    return output;
}

void WaypointGuidanceManager::update_altitude(double altitude_m) {
    if (altitude_m <= 0.0) {
        return;
    }

    if (!altitude_valid_) {
        current_altitude_ = altitude_m;
        altitude_valid_ = true;
        return;
    }

    current_altitude_ = config_.altitude_low_pass_alpha * current_altitude_ +
                        (1.0 - config_.altitude_low_pass_alpha) * altitude_m;
}

WaypointStatus WaypointGuidanceManager::status() const noexcept {
    return status_;
}

bool WaypointGuidanceManager::active() const noexcept {
    return waypoint_active_;
}

bool WaypointGuidanceManager::altitude_valid() const noexcept {
    return altitude_valid_;
}

double WaypointGuidanceManager::altitude() const noexcept {
    return current_altitude_;
}

vortex::utils::types::Pose WaypointGuidanceManager::current_goal() const {
    return follower_->waypoint_goal();
}

bool WaypointGuidanceManager::prepare_altitude_goal(
    vortex::utils::types::Waypoint& waypoint,
    const vortex::utils::types::Pose& current_pose) {
    if (!waypoint.keep_altitude) {
        return true;
    }

    if (!config_.altitude_control_enabled) {
        return true;
    }

    if (waypoint.desired_altitude <= 0.0) {
        return false;
    }

    if (!altitude_valid_) {
        // We do not yet know altitude above seabed.
        // Start with the waypoint's supplied z coordinate.
        return true;
    }

    waypoint.pose.z =
        current_pose.z + current_altitude_ - waypoint.desired_altitude;

    return true;
}

bool prepare_altitude_goal(vortex::utils::types::Waypoint& waypoint,
                           const vortex::utils::types::Pose& current_pose);

void WaypointGuidanceManager::recreate_follower() {
    const double time_step_s =
        static_cast<double>(config_.time_step.count()) / 1000.0;

    follower_ =
        std::make_unique<WaypointFollower>(config_.filter_params, time_step_s);
}

}  // namespace vortex::guidance

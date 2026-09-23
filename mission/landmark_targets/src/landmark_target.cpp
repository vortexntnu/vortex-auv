#include "landmark_targets/landmark_target.hpp"
#include <cmath>
#include <stdexcept>
#include <vortex/utils/waypoint_utils.hpp>

namespace vortex::mission {

Pose resolve_target(const MapLandmark& landmark, const TargetSpec& spec) {
    Eigen::Vector3d p_tool;
    Eigen::Quaterniond q_target;

    if (spec.frame == OffsetFrame::LANDMARK) {
        if (!landmark.has_orientation) {
            throw std::invalid_argument(
                "resolve_target: OffsetFrame::LANDMARK needs a landmark with "
                "orientation");
        }
        const Eigen::Quaterniond q_lm =
            landmark.pose.ori_quaternion().normalized();
        p_tool = landmark.pose.pos_vector() + q_lm * spec.offset.pos_vector();
        q_target =
            (q_lm * spec.offset.ori_quaternion().normalized()).normalized();
    } else {
        const Pose target = vortex::utils::waypoints::apply_pose_offset(
            landmark.pose, spec.offset);
        p_tool = target.pos_vector();
        q_target = target.ori_quaternion();
    }

    const Eigen::Vector3d p_base = p_tool - q_target * spec.tool_arm;
    return Pose::from_eigen(p_base, q_target);
}

LandmarkTarget::LandmarkTarget(TargetSpec spec, int landmark_id)
    : spec_(std::move(spec)), landmark_id_(landmark_id) {}

TargetStep LandmarkTarget::step(const std::optional<MapLandmark>& landmark,
                                const Pose& odom,
                                double now) {
    // Dead reckoning and LOST are terminal.
    if (phase_ != Phase::TRACKING) {
        return {std::nullopt, phase_};
    }

    std::optional<MapLandmark> lm = landmark;
    if (lm && lm->id != landmark_id_) {
        lm.reset();
    }
    const bool usable =
        lm && (spec_.frame != OffsetFrame::LANDMARK || lm->has_orientation);

    // How long has the landmark not been seen?
    double unseen_for = 0.0;
    if (lm) {
        unseen_for = std::max(0.0, now - lm->last_measurement);
    } else {
        if (!absent_since_) {
            absent_since_ = now;
        }
        unseen_for = now - *absent_since_;
    }
    if (usable && lm) {
        absent_since_.reset();
    }

    if (unseen_for > spec_.track_loss_timeout_sec) {
        phase_ = Phase::LOST;
        return {std::nullopt, phase_};
    }

    if (!usable) {
        // Keep the last goal until the timeout runs out.
        return {std::nullopt, phase_};
    }

    const Pose target = resolve_target(*lm, spec_);

    TargetStep out;
    bool send = false;
    if (!last_goal_) {
        send = true;
    } else if (!spec_.freeze) {
        const double moved =
            (target.pos_vector() - last_goal_->pos_vector()).norm();
        send = moved > spec_.resend_distance &&
               now - last_goal_time_ >= spec_.min_resend_interval_sec;
    }
    if (send) {
        last_goal_ = target;
        last_goal_time_ = now;
        out.send_goal = target;
    }

    if (spec_.freeze) {
        // The target is computed once; the node has nothing more to do.
        phase_ = Phase::DEAD_RECKONING;
    } else {
        const double dist = (odom.pos_vector() - target.pos_vector()).norm();
        if (std::isfinite(dist) && dist <= spec_.dead_reckoning_distance) {
            phase_ = Phase::DEAD_RECKONING;
        }
    }

    out.phase = phase_;
    return out;
}

}  // namespace vortex::mission

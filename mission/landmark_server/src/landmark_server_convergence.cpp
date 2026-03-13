#include <spdlog/spdlog.h>
#include <rclcpp_action/client.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

rclcpp_action::GoalResponse
LandmarkServerNode::handle_landmark_convergence_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const vortex_msgs::action::LandmarkConvergence::Goal>
        goal_msg) {
    if (goal_msg->convergence_threshold <= 0.0) {
        spdlog::warn(
            "LandmarkConvergence: reject, invalid convergence_threshold={:.3f}",
            goal_msg->convergence_threshold);
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_msg->dead_reckoning_threshold < 0.0) {
        spdlog::warn(
            "LandmarkConvergence: reject, invalid dead_reckoning_offset={:.3f}",
            goal_msg->dead_reckoning_threshold);
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_msg->track_loss_timeout_sec < 0.0) {
        spdlog::warn(
            "LandmarkConvergence: reject, invalid "
            "track_loss_timeout_sec={:.3f}",
            goal_msg->track_loss_timeout_sec);
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(5))) {
        spdlog::error(
            "LandmarkConvergence: reject, ReferenceFilter server not available "
            "after 5s");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (active_landmark_convergence_goal_ &&
        active_landmark_convergence_goal_->is_active()) {
        spdlog::warn(
            "LandmarkConvergence: preempting previous active session {}",
            convergence_session_id_);
        active_landmark_convergence_goal_->abort(
            std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
                build_convergence_result(false)));
    }

    cancel_reference_filter_goal();

    spdlog::info(
        "LandmarkConvergence: goal accepted — type={}, subtype={}, "
        "conv_threshold={:.3f}, dr_threshold={:.3f}, "
        "track_loss_timeout={:.1f}s",
        goal_msg->type.value, goal_msg->subtype.value,
        goal_msg->convergence_threshold, goal_msg->dead_reckoning_threshold,
        goal_msg->track_loss_timeout_sec);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void LandmarkServerNode::handle_landmark_convergence_accepted(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> goal_handle) {
    active_landmark_convergence_goal_ = goal_handle;

    const auto goal = active_landmark_convergence_goal_->get_goal();

    ++convergence_session_id_;

    convergence_active_ = true;
    convergence_dead_reckoning_handoff_ = false;
    convergence_last_known_track_.reset();
    convergence_track_lost_ = false;
    rf_state_ = RFState::IDLE;

    spdlog::info(
        "Starting convergence session {}: type={}, subtype={}, "
        "threshold={:.3f}, dr_offset={:.3f}, track_loss_timeout_sec={:.3f}",
        convergence_session_id_, convergence_goal()->type.value,
        convergence_goal()->subtype.value,
        convergence_goal()->convergence_threshold,
        convergence_goal()->dead_reckoning_threshold,
        convergence_goal()->track_loss_timeout_sec);

    const auto track = get_convergence_track();
    if (!track) {
        convergence_track_lost_ = true;
        convergence_track_lost_since_ = now();
        spdlog::warn(
            "Convergence session {}: no track at start (type={}, subtype={}), "
            "waiting up to {:.1f}s for track to appear.",
            convergence_session_id_, convergence_goal()->type.value,
            convergence_goal()->subtype.value,
            convergence_goal()->track_loss_timeout_sec);
        return;
    }

    try {
        const auto target =
            compute_target_pose(*track, convergence_goal()->convergence_offset);
        spdlog::info(
            "Convergence session {}: initial target = ({:.3f}, {:.3f}, {:.3f})",
            convergence_session_id_, target.position.x, target.position.y,
            target.position.z);
        send_reference_filter_goal(
            make_rf_goal(target, convergence_goal()->convergence_threshold),
            convergence_session_id_);
    } catch (const std::exception& e) {
        spdlog::error("Failed to compute initial convergence target: {}",
                      e.what());
        goal_handle->abort(
            std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
                build_convergence_result(false)));
        convergence_active_ = false;
    }
}

rclcpp_action::CancelResponse
LandmarkServerNode::handle_landmark_convergence_cancel(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> /*goal_handle*/) {
    spdlog::info("Canceling convergence session {}", convergence_session_id_);

    cancel_reference_filter_goal();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarkServerNode::convergence_update() {
    if (!convergence_active_)
        return;

    if (!convergence_goal_active()) {
        spdlog::warn(
            "Convergence session {}: goal no longer active in "
            "convergence_update "
            "(convergence_active_={}, goal_handle_valid={}, is_active={}), "
            "clearing state",
            convergence_session_id_, convergence_active_,
            active_landmark_convergence_goal_ != nullptr,
            active_landmark_convergence_goal_
                ? active_landmark_convergence_goal_->is_active()
                : false);
        convergence_active_ = false;
        return;
    }

    if (convergence_dead_reckoning_handoff_) {
        // Waiting for the RF goal to finish — log a periodic heartbeat.
        static uint32_t dr_tick = 0;
        if (++dr_tick % 50 == 0) {
            spdlog::info(
                "Convergence session {}: dead-reckoning active, "
                "waiting for RF to finish (type={}, subtype={})",
                convergence_session_id_, convergence_goal()->type.value,
                convergence_goal()->subtype.value);
        }
        return;
    }

    const auto track = get_convergence_track();
    if (!track) {
        convergence_handle_track_loss();
        return;
    }

    if (convergence_track_lost_) {
        spdlog::info("Convergence: track reacquired (type={}, subtype={})",
                     convergence_goal()->type.value,
                     convergence_goal()->subtype.value);
        convergence_track_lost_ = false;
    }

    convergence_last_known_track_ = *track;

    convergence_update_target(*track);
    convergence_check_dr_handoff();
}

bool LandmarkServerNode::convergence_track_timeout() const {
    if (!convergence_track_lost_)
        return false;

    const double elapsed = (now() - convergence_track_lost_since_).seconds();
    return elapsed >= convergence_goal()->track_loss_timeout_sec;
}

void LandmarkServerNode::convergence_abort_track_loss() {
    spdlog::error(
        "Convergence: track loss timeout ({:.1f} s) exceeded, "
        "aborting (type={}, subtype={})",
        convergence_goal()->track_loss_timeout_sec,
        convergence_goal()->type.value, convergence_goal()->subtype.value);

    cancel_reference_filter_goal();

    active_landmark_convergence_goal_->abort(
        std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
            build_convergence_result(false)));
    convergence_active_ = false;
}

void LandmarkServerNode::convergence_handle_track_loss() {
    if (!convergence_track_lost_) {
        convergence_track_lost_ = true;
        convergence_track_lost_since_ = now();
        spdlog::warn(
            "Convergence: track lost (type={}, subtype={}), "
            "starting {:.1f} s timeout",
            convergence_goal()->type.value, convergence_goal()->subtype.value,
            convergence_goal()->track_loss_timeout_sec);
        return;
    }

    if (convergence_track_timeout()) {
        convergence_abort_track_loss();
    }
}

void LandmarkServerNode::convergence_update_target(
    const vortex::filtering::Track& track) {
    switch (rf_state_) {
        case RFState::PENDING:
            // async_send_goal() fired but goal_response_callback hasn't arrived
            // yet — perfectly normal, nothing to do this tick.
            return;

        case RFState::IDLE:
            spdlog::error(
                "Convergence session {}: RF goal unexpectedly IDLE "
                "(type={}, subtype={}), aborting",
                convergence_session_id_, convergence_goal()->type.value,
                convergence_goal()->subtype.value);
            active_landmark_convergence_goal_->abort(
                std::make_shared<
                    vortex_msgs::action::LandmarkConvergence::Result>(
                    build_convergence_result(false)));
            convergence_active_ = false;
            return;

        case RFState::ACTIVE:
            break;
    }

    try {
        geometry_msgs::msg::PoseStamped target;
        target.header.stamp = now();
        target.header.frame_id = target_frame_;
        target.pose =
            compute_target_pose(track, convergence_goal()->convergence_offset);
        reference_pose_pub_->publish(target);
    } catch (const std::exception& e) {
        spdlog::warn("Convergence session {}: target recompute failed: {}",
                     convergence_session_id_, e.what());
    }
}

void LandmarkServerNode::convergence_check_dr_handoff() {
    if (!convergence_last_known_track_)
        return;

    std::optional<geometry_msgs::msg::Point> odom_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        odom_pos = last_odom_position_;
    }

    if (!odom_pos) {
        spdlog::warn("Dead reckoning check: no odometry received yet on '{}'",
                     this->get_parameter("topics.odom").as_string());
        return;
    }

    // Compare against the *target* (landmark + offset), not the raw landmark
    // position. Using the landmark position directly means DR is only triggered
    // when the AUV is inside the landmark itself, which is rarely the intended
    // behaviour when a non-zero convergence_offset is used.
    geometry_msgs::msg::Pose target_pose;
    try {
        target_pose =
            compute_target_pose(*convergence_last_known_track_,
                                convergence_goal()->convergence_offset);
    } catch (const std::exception& e) {
        spdlog::warn("Dead reckoning check: compute_target_pose failed: {}",
                     e.what());
        return;
    }

    const auto& cur = *odom_pos;
    const double dx = cur.x - target_pose.position.x;
    const double dy = cur.y - target_pose.position.y;
    const double dz = cur.z - target_pose.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (!std::isfinite(dist)) {
        spdlog::warn("Invalid distance in dead reckoning check");
        return;
    }

    if (dist <= convergence_goal()->dead_reckoning_threshold) {
        convergence_dead_reckoning_handoff_ = true;
        spdlog::info(
            "Convergence session {}: dead-reckoning handoff triggered — "
            "dist_to_target={:.3f} <= dr_threshold={:.3f}, "
            "target=({:.3f}, {:.3f}, {:.3f}), "
            "auv=({:.3f}, {:.3f}, {:.3f})",
            convergence_session_id_, dist,
            convergence_goal()->dead_reckoning_threshold,
            target_pose.position.x, target_pose.position.y,
            target_pose.position.z, cur.x, cur.y, cur.z);
    } else {
        spdlog::debug(
            "Convergence session {}: DR check — "
            "dist_to_target={:.3f}, dr_threshold={:.3f}",
            convergence_session_id_, dist,
            convergence_goal()->dead_reckoning_threshold);
    }
}

vortex_msgs::action::ReferenceFilterWaypoint::Goal
LandmarkServerNode::make_rf_goal(const geometry_msgs::msg::Pose& target,
                                 double convergence_threshold) const {
    vortex_msgs::action::ReferenceFilterWaypoint::Goal rf_goal;
    vortex_msgs::msg::Waypoint wp;
    wp.pose = target;
    // Use ONLY_POSITION: the landmark's orientation should not be imposed on
    // the AUV. Converging on position alone is sufficient, and it prevents the
    // RF from never succeeding when the landmark has an arbitrary orientation
    // (e.g. a pipe lying at 90 degrees).
    wp.mode = vortex_msgs::msg::Waypoint::ONLY_POSITION;
    rf_goal.waypoint = wp;
    rf_goal.convergence_threshold = convergence_threshold;
    return rf_goal;
}

void LandmarkServerNode::send_reference_filter_goal(
    const vortex_msgs::action::ReferenceFilterWaypoint::Goal& goal_msg,
    uint64_t session_id) {
    cancel_reference_filter_goal();

    rf_state_ = RFState::PENDING;

    rclcpp_action::Client<RF>::SendGoalOptions options;

    options.goal_response_callback =
        [this, session_id](ReferenceFilterGoalHandle::SharedPtr gh) {
            if (!gh) {
                spdlog::error(
                    "Convergence session {}: ReferenceFilter goal REJECTED",
                    session_id);
                rf_state_ = RFState::IDLE;
                if (session_id == convergence_session_id_ &&
                    convergence_goal_active()) {
                    active_landmark_convergence_goal_->abort(
                        std::make_shared<
                            vortex_msgs::action::LandmarkConvergence::Result>(
                            build_convergence_result(false)));
                    convergence_active_ = false;
                }
                return;
            }

            if (session_id != convergence_session_id_) {
                spdlog::warn(
                    "Convergence: stale RF goal response for session {} "
                    "(current={}), canceling it",
                    session_id, convergence_session_id_);
                rf_state_ = RFState::IDLE;
                reference_filter_client_->async_cancel_goal(gh);
                return;
            }

            spdlog::info(
                "Convergence session {}: ReferenceFilter goal ACCEPTED",
                session_id);
            rf_state_ = RFState::ACTIVE;
            active_reference_filter_goal_ = gh;
        };

    options.result_callback =
        [this,
         session_id](const ReferenceFilterGoalHandle::WrappedResult& res) {
            if (session_id != convergence_session_id_)
                return;

            rf_state_ = RFState::IDLE;
            active_reference_filter_goal_.reset();

            if (!convergence_active_ || !active_landmark_convergence_goal_) {
                spdlog::warn(
                    "Convergence session {}: RF result arrived but "
                    "convergence already inactive — ignoring",
                    session_id);
                return;
            }

            spdlog::info("Convergence session {}: RF result — code={}",
                         session_id, static_cast<int>(res.code));
            handle_rf_result(res.code);
        };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

void LandmarkServerNode::handle_rf_result(rclcpp_action::ResultCode code) {
    using RC = rclcpp_action::ResultCode;
    auto make_result = [this](bool success) {
        return std::make_shared<
            vortex_msgs::action::LandmarkConvergence::Result>(
            build_convergence_result(success));
    };

    switch (code) {
        case RC::SUCCEEDED:
            spdlog::info("Convergence succeeded (RF succeeded)");
            active_landmark_convergence_goal_->succeed(make_result(true));
            break;

        case RC::CANCELED:
            spdlog::info("Convergence canceled (RF canceled)");
            active_landmark_convergence_goal_->canceled(make_result(false));
            break;

        case RC::ABORTED:
            spdlog::warn("Convergence aborted (RF aborted)");
            active_landmark_convergence_goal_->abort(make_result(false));
            break;

        default:
            spdlog::error("Convergence aborted (unknown RF result)");
            active_landmark_convergence_goal_->abort(make_result(false));
            break;
    }

    convergence_active_ = false;
}

std::optional<vortex::filtering::Track>
LandmarkServerNode::get_convergence_track() const {
    const auto tracks =
        track_manager_->get_tracks_by_type(vortex::filtering::LandmarkClassKey{
            convergence_goal()->type.value, convergence_goal()->subtype.value});
    if (tracks.empty())
        return std::nullopt;
    return *tracks.front();
}

bool LandmarkServerNode::convergence_goal_active() const {
    return convergence_active_ && active_landmark_convergence_goal_ &&
           active_landmark_convergence_goal_->is_active();
}

void LandmarkServerNode::cancel_reference_filter_goal() {
    rf_state_ = RFState::IDLE;

    if (!active_reference_filter_goal_)
        return;

    spdlog::info("Convergence session {}: canceling active RF goal",
                 convergence_session_id_);
    const auto goal_to_cancel = active_reference_filter_goal_;
    active_reference_filter_goal_.reset();
    reference_filter_client_->async_cancel_goal(goal_to_cancel);
}

vortex_msgs::action::LandmarkConvergence::Result
LandmarkServerNode::build_convergence_result(bool success) const {
    vortex_msgs::action::LandmarkConvergence::Result res;
    res.success = success;
    res.landmark_valid = convergence_last_known_track_.has_value();

    if (!convergence_last_known_track_) {
        return res;
    }

    const auto& track = *convergence_last_known_track_;
    res.landmark = track_to_landmark_msg(track);

    return res;
}

geometry_msgs::msg::Pose LandmarkServerNode::compute_target_pose(
    const vortex::filtering::Track& track,
    const geometry_msgs::msg::Pose& convergence_offset) {
    const auto landmark_pose = track.to_pose();

    const Eigen::Vector3d p_landmark = landmark_pose.pos_vector();
    const Eigen::Quaterniond q_landmark =
        landmark_pose.ori_quaternion().normalized();

    const Eigen::Vector3d p_offset(convergence_offset.position.x,
                                   convergence_offset.position.y,
                                   convergence_offset.position.z);

    const Eigen::Quaterniond q_offset =
        Eigen::Quaterniond(
            convergence_offset.orientation.w, convergence_offset.orientation.x,
            convergence_offset.orientation.y, convergence_offset.orientation.z)
            .normalized();

    const Eigen::Vector3d p_target = p_landmark + q_landmark * p_offset;

    const Eigen::Quaterniond q_target = (q_landmark * q_offset).normalized();

    spdlog::debug(
        "compute_target_pose: "
        "landmark=({:.3f}, {:.3f}, {:.3f}) "
        "q_landmark=({:.3f}, {:.3f}, {:.3f}, {:.3f}) "
        "offset=({:.3f}, {:.3f}, {:.3f}) "
        "→ target=({:.3f}, {:.3f}, {:.3f})",
        p_landmark.x(), p_landmark.y(), p_landmark.z(), q_landmark.x(),
        q_landmark.y(), q_landmark.z(), q_landmark.w(), p_offset.x(),
        p_offset.y(), p_offset.z(), p_target.x(), p_target.y(), p_target.z());

    return vortex::utils::ros_conversions::eigen_to_pose_msg(p_target,
                                                             q_target);
}

}  // namespace vortex::mission

#include "waypoint_manager/waypoint_manager_ros.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>

namespace vortex::mission {

WaypointManagerNode::WaypointManagerNode(const rclcpp::NodeOptions& options)
    : Node("waypoint_manager_node", options) {
    set_reference_action_client();
    set_waypoint_action_server();
    set_waypoint_service_server();

    spdlog::info("WaypointManagerNode started");
}

WaypointManagerNode::~WaypointManagerNode() {
    if (active_action_goal_ && (active_action_goal_->is_active() ||
                                active_action_goal_->is_canceling())) {
        try {
            auto res = construct_result(false);
            active_action_goal_->abort(res);
        } catch (...) {
        }
    }

    if (active_reference_filter_goal_) {
        try {
            reference_filter_client_->async_cancel_goal(
                active_reference_filter_goal_);
        } catch (...) {
        }
        active_reference_filter_goal_.reset();
    }
}

// ---------------------------------------------------------
// SETUP INTERFACES
// ---------------------------------------------------------

void WaypointManagerNode::set_reference_action_client() {
    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(this,
                                                            "reference_filter");

    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilter server not ready");
    }
}

void WaypointManagerNode::set_waypoint_action_server() {
    waypoint_action_server_ = rclcpp_action::create_server<WaypointManager>(
        this, "waypoint_manager",

        [this](auto goal_id, auto goal) {
            return handle_waypoint_goal(goal_id, goal);
        },

        [this](auto goal_id) { return handle_waypoint_cancel(goal_id); },

        [this](auto goal_handle) {
            return handle_waypoint_accepted(goal_handle);
        });
}

void WaypointManagerNode::set_waypoint_service_server() {
    waypoint_service_server_ =
        this->create_service<vortex_msgs::srv::SendWaypoints>(
            "waypoint_addition",
            std::bind(
                &WaypointManagerNode::handle_send_waypoints_service_request,
                this, std::placeholders::_1, std::placeholders::_2));
}

// ---------------------------------------------------------
// HELPERS
// ---------------------------------------------------------

std::shared_ptr<vortex_msgs::action::WaypointManager_Result>
WaypointManagerNode::construct_result(bool success) const {
    auto result =
        std::make_shared<vortex_msgs::action::WaypointManager_Result>();
    result->success = success;
    result->pose_valid = has_reference_pose_;
    if (has_reference_pose_) {
        result->final_pose = vortex::utils::ros_conversions::to_pose_msg(
            latest_ref_feedback_.reference);
    }
    return result;
}

void WaypointManagerNode::cleanup_mission_state() {
    waypoints_.clear();
    current_index_ = 0;
    persistent_action_mode_active_ = false;
    priority_mode_active_ = false;
    has_reference_pose_ = false;

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    active_action_goal_.reset();
}

void WaypointManagerNode::send_next_reference_filter_goal() {
    if (current_index_ >= waypoints_.size()) {
        if (!persistent_action_mode_active_ && active_action_goal_ &&
            active_action_goal_->is_active()) {
            auto wm_res = construct_result(true);
            active_action_goal_->succeed(wm_res);
            cleanup_mission_state();
        }
        return;
    }

    ReferenceFilterAction::Goal rf_goal;
    rf_goal.waypoint = waypoints_[current_index_];
    rf_goal.convergence_threshold = convergence_threshold_;

    send_reference_filter_goal(rf_goal);
}

// ---------------------------------------------------------
// WAYPOINT MANAGER ACTION SERVER
// ---------------------------------------------------------

rclcpp_action::GoalResponse WaypointManagerNode::handle_waypoint_goal(
    const rclcpp_action::GoalUUID& /*goal_uuid*/,
    std::shared_ptr<const WaypointManager::Goal> goal) {
    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto wp_res = construct_result(false);
        active_action_goal_->abort(wp_res);
    }

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    ++mission_id_;

    waypoints_ = goal->waypoints;
    current_index_ = 0;
    persistent_action_mode_active_ = goal->persistent;
    priority_mode_active_ = false;
    has_reference_pose_ = false;
    convergence_threshold_ = goal->convergence_threshold;

    if (waypoints_.empty() && !persistent_action_mode_active_) {
        spdlog::warn(
            "WaypointManager: received empty waypoint list and non-persistent "
            "mode");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle) {
    spdlog::info("WaypointManager: action goal accepted");
    active_action_goal_ = goal_handle;

    send_next_reference_filter_goal();
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle> /*goal_handle*/) {
    spdlog::info("WaypointManagerAction: cancel requested");

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------
// WAYPOINT MANAGER SERVICE SERVER
// ---------------------------------------------------------

void WaypointManagerNode::handle_send_waypoints_service_request(
    const std::shared_ptr<vortex_msgs::srv::SendWaypoints::Request> request,
    std::shared_ptr<vortex_msgs::srv::SendWaypoints::Response> response) {
    if (!persistent_action_mode_active_ || !active_action_goal_ ||
        !active_action_goal_->is_active()) {
        response->success = false;
        return;
    }

    if (priority_mode_active_ && !request->take_priority &&
        current_index_ < waypoints_.size()) {
        response->success = false;
        return;
    }

    priority_mode_active_ = request->take_priority;

    if (request->overwrite_prior_waypoints) {
        mission_id_++;
        waypoints_ = request->waypoints;
        current_index_ = 0;
        has_reference_pose_ = false;

        if (active_reference_filter_goal_) {
            reference_filter_client_->async_cancel_goal(
                active_reference_filter_goal_);
            active_reference_filter_goal_.reset();
        }

        if (!waypoints_.empty()) {
            send_next_reference_filter_goal();
        }

        response->success = true;
        return;
    }

    waypoints_.insert(waypoints_.end(), request->waypoints.begin(),
                      request->waypoints.end());

    if (!active_reference_filter_goal_ && current_index_ < waypoints_.size()) {
        send_next_reference_filter_goal();
    }

    response->success = true;
}

// ---------------------------------------------------------
// REFERENCE FILTER ACTION CLIENT
// ---------------------------------------------------------

void WaypointManagerNode::send_reference_filter_goal(
    const ReferenceFilterAction::Goal& goal_msg) {
    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    const std::uint64_t this_mission = mission_id_;

    rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions options;

    options.goal_response_callback =
        [this, this_mission](ReferenceFilterGoalHandle::SharedPtr gh) {
            if (!gh) {
                spdlog::warn("ReferenceFilter goal rejected");
                return;
            }

            if (this_mission == mission_id_) {
                active_reference_filter_goal_ = gh;
            } else {
                spdlog::info(
                    "RF goal response for old mission, ignoring handle");
            }
        };

    options.feedback_callback =
        [this, this_mission](
            ReferenceFilterGoalHandle::SharedPtr,
            const std::shared_ptr<const ReferenceFilterAction::Feedback> fb) {
            if (this_mission != mission_id_) {
                return;
            }

            latest_ref_feedback_ = *fb;
            has_reference_pose_ = true;

            if (!active_action_goal_ || !active_action_goal_->is_active())
                return;

            geometry_msgs::msg::Pose robot_pose =
                vortex::utils::ros_conversions::to_pose_msg(fb->reference);

            if (current_index_ < waypoints_.size()) {
                auto wm_fb = std::make_shared<WaypointManager::Feedback>();
                wm_fb->current_pose = robot_pose;
                wm_fb->current_waypoint = waypoints_[current_index_];
                active_action_goal_->publish_feedback(wm_fb);
            }
        };

    options
        .result_callback = [this, this_mission](
                               const ReferenceFilterGoalHandle::WrappedResult&
                                   res) {
        if (this_mission != mission_id_) {
            spdlog::info(
                "ReferenceFilter result received for old mission, ignoring.");
            return;
        }

        active_reference_filter_goal_.reset();

        if (!active_action_goal_) {
            spdlog::info(
                "ReferenceFilter result received but no active WM goal");
            return;
        }

        const bool wm_canceling = active_action_goal_->is_canceling();
        const bool wm_active = active_action_goal_->is_active();

        switch (res.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: {
                spdlog::info("ReferenceFilter goal reached waypoint");

                if (wm_canceling) {
                    bool action_success = false;
                    if (persistent_action_mode_active_ &&
                        current_index_ >= waypoints_.size()) {
                        action_success = true;
                    }

                    auto wp_res = construct_result(action_success);

                    active_action_goal_->canceled(wp_res);

                    cleanup_mission_state();
                } else {
                    current_index_++;
                    send_next_reference_filter_goal();
                }
                break;
            }

            case rclcpp_action::ResultCode::CANCELED: {
                spdlog::info("ReferenceFilter goal cancelled");

                if (wm_canceling && wm_active) {
                    bool action_success = false;
                    if (persistent_action_mode_active_ &&
                        current_index_ >= waypoints_.size()) {
                        action_success = true;
                    }

                    auto wp_res = construct_result(action_success);

                    active_action_goal_->canceled(wp_res);
                    cleanup_mission_state();
                }
                break;
            }

            case rclcpp_action::ResultCode::ABORTED: {
                spdlog::warn("ReferenceFilter goal aborted unexpectedly");
                if (wm_active) {
                    auto wp_res = construct_result(false);
                    active_action_goal_->abort(wp_res);
                    cleanup_mission_state();
                }
                break;
            }

            default:
                spdlog::error(
                    "ReferenceFilter goal returned unknown result code");
                break;
        }
    };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointManagerNode)

}  // namespace vortex::mission

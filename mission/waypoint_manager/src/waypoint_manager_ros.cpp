#include "waypoint_manager/waypoint_manager_ros.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "waypoint_manager/frame_resolver.hpp"

namespace vortex::mission {

namespace {

bool is_finite_pose(const geometry_msgs::msg::Pose& p) {
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
           std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
           std::isfinite(p.orientation.w);
}

}  // namespace

WaypointManagerNode::WaypointManagerNode(const rclcpp::NodeOptions& options)
    : Node("waypoint_manager_node", options) {
    set_pose_subscription();
    set_reference_action_client();
    set_waypoint_action_server();
    set_waypoint_service_server();
    setup_reset_subscription();
    setup_debug_publisher();

    spdlog::info("WaypointManagerNode started");
}

WaypointManagerNode::~WaypointManagerNode() {
    if (active_action_goal_ && (active_action_goal_->is_active() ||
                                active_action_goal_->is_canceling())) {
        try {
            auto res = construct_result(WaypointManager::Result::PREEMPTED,
                                        "waypoint manager shutting down");
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

void WaypointManagerNode::set_pose_subscription() {
    const std::string pose_topic =
        this->declare_parameter<std::string>("topics.pose", "pose");
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, vortex::utils::qos_profiles::sensor_data_profile(1),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = vortex::utils::ros_conversions::ros_pose_to_pose(
                msg->pose.pose);
        });
}

void WaypointManagerNode::set_reference_action_client() {
    const std::string action_name = this->declare_parameter<std::string>(
        "action_servers.reference_filter", "reference_filter");
    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(this, action_name);

    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilter server not ready");
    }
}

void WaypointManagerNode::set_waypoint_action_server() {
    std::string action_name =
        this->declare_parameter<std::string>("action_servers.waypoint_manager");
    waypoint_action_server_ = rclcpp_action::create_server<WaypointManager>(
        this, action_name,

        [this](auto goal_id, auto goal) {
            return handle_waypoint_goal(goal_id, goal);
        },

        [this](auto goal_id) { return handle_waypoint_cancel(goal_id); },

        [this](auto goal_handle) {
            return handle_waypoint_accepted(goal_handle);
        });
}

void WaypointManagerNode::setup_debug_publisher() {
    const std::string mode_str = this->declare_parameter<std::string>(
        "debug.waypoint_publish_mode", "none");

    if (mode_str == "timer") {
        debug_mode_ = DebugPublishMode::timer;
    } else if (mode_str == "on_new_waypoint") {
        debug_mode_ = DebugPublishMode::on_new_waypoint;
    } else {
        debug_mode_ = DebugPublishMode::none;
        return;
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    const std::string debug_topic_name = this->declare_parameter<std::string>(
        "debug.waypoint_topic_name", "debug/waypoint");

    debug_waypoint_pub_ = this->create_publisher<vortex_msgs::msg::Waypoint>(
        debug_topic_name, qos);

    if (debug_mode_ == DebugPublishMode::timer) {
        debug_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(100),
                                    [this]() { publish_current_waypoint(); });
    }

    spdlog::info("Waypoint debug publisher active (mode: {})", mode_str);
}

void WaypointManagerNode::publish_current_waypoint() {
    if (!debug_waypoint_pub_ || current_index_ >= waypoints_.size()) {
        return;
    }
    debug_waypoint_pub_->publish(waypoints_[current_index_]);
}

void WaypointManagerNode::setup_reset_subscription() {
    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "mission/wipe", vortex::utils::qos_profiles::reliable_profile(1),
        [this](std_msgs::msg::Empty::ConstSharedPtr msg) {
            on_system_reset(msg);
        });
}

void WaypointManagerNode::on_system_reset(
    std_msgs::msg::Empty::ConstSharedPtr) {
    if (active_action_goal_ && (active_action_goal_->is_active() ||
                                active_action_goal_->is_canceling())) {
        auto res = construct_result(WaypointManager::Result::CANCELED,
                                    "system reset (mission/wipe)");
        active_action_goal_->abort(res);
    }
    cleanup_mission_state();
    spdlog::info("WaypointManager: reset complete");
}

void WaypointManagerNode::set_waypoint_service_server() {
    std::string service_name =
        this->declare_parameter<std::string>("services.waypoint_addition");
    waypoint_service_server_ =
        this->create_service<vortex_msgs::srv::SendWaypoints>(
            service_name,
            std::bind(
                &WaypointManagerNode::handle_send_waypoints_service_request,
                this, std::placeholders::_1, std::placeholders::_2));
}

// ---------------------------------------------------------
// HELPERS
// ---------------------------------------------------------

std::shared_ptr<vortex_msgs::action::WaypointManager_Result>
WaypointManagerNode::construct_result(uint8_t outcome,
                                      const std::string& message) const {
    auto result =
        std::make_shared<vortex_msgs::action::WaypointManager_Result>();
    result->success = (outcome == WaypointManager::Result::SUCCEEDED);
    result->outcome = outcome;
    result->message = message;
    // Index of the last waypoint that was reached; -1 if none.
    result->reached_index = static_cast<int32_t>(current_index_) - 1;
    return result;
}

void WaypointManagerNode::cleanup_mission_state() {
    // Late responses for goals sent before this point are then recognised as
    // belonging to an old mission.
    ++mission_id_;
    waypoints_.clear();
    current_index_ = 0;
    persistent_action_mode_active_ = false;
    priority_mode_active_ = false;

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    active_action_goal_.reset();
}

void WaypointManagerNode::finish_active_goal(uint8_t outcome,
                                             const std::string& message) {
    if (!active_action_goal_) {
        return;
    }
    auto result = construct_result(outcome, message);
    if (active_action_goal_->is_active() ||
        active_action_goal_->is_canceling()) {
        switch (outcome) {
            case WaypointManager::Result::SUCCEEDED:
                active_action_goal_->succeed(result);
                break;
            case WaypointManager::Result::CANCELED:
                if (active_action_goal_->is_canceling()) {
                    active_action_goal_->canceled(result);
                } else {
                    active_action_goal_->abort(result);
                }
                break;
            default:
                active_action_goal_->abort(result);
                break;
        }
    }
    spdlog::info("WaypointManager: goal finished (outcome {}): {}", outcome,
                 message);
    cleanup_mission_state();
}

void WaypointManagerNode::send_next_reference_filter_goal() {
    if (current_index_ >= waypoints_.size()) {
        if (!persistent_action_mode_active_ && active_action_goal_ &&
            active_action_goal_->is_active()) {
            finish_active_goal(WaypointManager::Result::SUCCEEDED,
                               "all waypoints reached");
        }
        return;
    }

    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto wm_fb = std::make_shared<WaypointManager::Feedback>();
        wm_fb->current_waypoint = waypoints_[current_index_];
        wm_fb->current_index = static_cast<int32_t>(current_index_);
        active_action_goal_->publish_feedback(wm_fb);
    }

    if (debug_mode_ == DebugPublishMode::on_new_waypoint) {
        publish_current_waypoint();
    }

    ReferenceFilterAction::Goal rf_goal;
    rf_goal.waypoint = waypoints_[current_index_];
    rf_goal.convergence_threshold = convergence_threshold_;

    send_reference_filter_goal(rf_goal);
}

std::string WaypointManagerNode::validate_and_resolve_goal(
    const WaypointManager::Goal& goal,
    std::vector<vortex_msgs::msg::Waypoint>& resolved) const {
    for (std::size_t i = 0; i < goal.waypoints.size(); ++i) {
        if (!is_finite_pose(goal.waypoints[i].pose)) {
            return "waypoint " + std::to_string(i) +
                   " has NaN or inf in its pose";
        }
    }

    GoalFrame frame = GoalFrame::WORLD;
    switch (goal.frame) {
        case WaypointManager::Goal::WORLD:
            break;
        case WaypointManager::Goal::BODY_RELATIVE:
            frame = GoalFrame::BODY_RELATIVE;
            break;
        case WaypointManager::Goal::WORLD_RELATIVE:
            frame = GoalFrame::WORLD_RELATIVE;
            break;
        default:
            return "unknown frame " + std::to_string(goal.frame);
    }

    resolved = goal.waypoints;
    if (frame == GoalFrame::WORLD) {
        return "";
    }

    std::optional<vortex::utils::types::Pose> start;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        start = current_pose_;
    }
    if (!start) {
        return "relative frame requested but no vehicle pose received yet";
    }

    for (auto& wp : resolved) {
        const auto absolute = resolve_pose(
            vortex::utils::ros_conversions::ros_pose_to_pose(wp.pose), frame,
            *start);
        wp.pose = vortex::utils::ros_conversions::to_pose_msg(absolute);
    }
    return "";
}

// ---------------------------------------------------------
// WAYPOINT MANAGER ACTION SERVER
// ---------------------------------------------------------

rclcpp_action::GoalResponse WaypointManagerNode::handle_waypoint_goal(
    const rclcpp_action::GoalUUID& /*goal_uuid*/,
    std::shared_ptr<const WaypointManager::Goal> goal) {
    if (goal->waypoints.empty() && !goal->persistent) {
        spdlog::warn(
            "WaypointManager: received empty waypoint list and non-persistent "
            "mode");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();

    std::vector<vortex_msgs::msg::Waypoint> resolved;
    const std::string error = validate_and_resolve_goal(*goal, resolved);
    if (!error.empty()) {
        // Leave a running mission untouched; only the new goal is invalid.
        spdlog::warn("WaypointManager: invalid goal: {}", error);
        auto result = std::make_shared<WaypointManager::Result>();
        result->success = false;
        result->outcome = WaypointManager::Result::INVALID_GOAL;
        result->message = error;
        result->reached_index = -1;
        goal_handle->abort(result);
        return;
    }

    spdlog::info("WaypointManager: action goal accepted");

    // A new goal replaces the running one. The reference filter is NOT
    // cancelled: it retargets to the new goal and keeps its velocity, so the
    // vehicle does not stop in between.
    if (active_action_goal_ && (active_action_goal_->is_active() ||
                                active_action_goal_->is_canceling())) {
        auto res = construct_result(WaypointManager::Result::PREEMPTED,
                                    "replaced by a newer goal");
        active_action_goal_->abort(res);
    }
    active_reference_filter_goal_.reset();

    ++mission_id_;
    waypoints_ = std::move(resolved);
    current_index_ = 0;
    persistent_action_mode_active_ = goal->persistent;
    priority_mode_active_ = false;
    convergence_threshold_ = goal->convergence_threshold;
    active_action_goal_ = goal_handle;

    send_next_reference_filter_goal();
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle> /*goal_handle*/) {
    spdlog::info("WaypointManagerAction: cancel requested");

    if (active_reference_filter_goal_) {
        // The result callback finishes the goal once the filter has stopped.
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    } else {
        // Nothing to wait for (e.g. persistent goal without waypoints). The
        // goal only enters CANCELING after this callback returns, so finish
        // it from a one-shot timer.
        cancel_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
                cancel_timer_->cancel();
                if (active_action_goal_ &&
                    active_action_goal_->is_canceling()) {
                    finish_active_goal(WaypointManager::Result::CANCELED,
                                       "canceled by client");
                }
            });
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

        if (waypoints_.empty()) {
            if (active_reference_filter_goal_) {
                reference_filter_client_->async_cancel_goal(
                    active_reference_filter_goal_);
                active_reference_filter_goal_.reset();
            }
        } else {
            // Retarget without cancelling the running reference filter goal.
            active_reference_filter_goal_.reset();
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
    const std::uint64_t this_mission = mission_id_;

    rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions options;

    options.goal_response_callback =
        [this, this_mission](ReferenceFilterGoalHandle::SharedPtr gh) {
            if (!gh) {
                spdlog::warn("ReferenceFilter goal rejected");
                if (this_mission == mission_id_ && active_action_goal_) {
                    finish_active_goal(
                        WaypointManager::Result::REFERENCE_FILTER_ABORTED,
                        "reference filter rejected the goal");
                }
                return;
            }

            if (this_mission == mission_id_) {
                active_reference_filter_goal_ = gh;
            } else if (!active_action_goal_) {
                // The mission ended (cancel/reset) while this goal was in
                // flight: stop the filter. A newer goal, in contrast, has
                // already replaced this one in the filter.
                reference_filter_client_->async_cancel_goal(gh);
            } else {
                spdlog::info(
                    "RF goal response for old mission, ignoring handle");
            }
        };

    options.result_callback =
        [this,
         this_mission](const ReferenceFilterGoalHandle::WrappedResult& res) {
            if (this_mission != mission_id_) {
                spdlog::info(
                    "ReferenceFilter result received for old mission, "
                    "ignoring.");
                return;
            }

            active_reference_filter_goal_.reset();

            if (!active_action_goal_) {
                spdlog::info(
                    "ReferenceFilter result received but no active WM goal");
                return;
            }

            const bool wm_canceling = active_action_goal_->is_canceling();

            switch (res.code) {
                case rclcpp_action::ResultCode::SUCCEEDED: {
                    spdlog::info("ReferenceFilter goal reached waypoint");
                    current_index_++;

                    if (wm_canceling) {
                        finish_active_goal(WaypointManager::Result::CANCELED,
                                           "canceled by client");
                    } else {
                        send_next_reference_filter_goal();
                    }
                    break;
                }

                case rclcpp_action::ResultCode::CANCELED: {
                    spdlog::info("ReferenceFilter goal cancelled");
                    if (wm_canceling) {
                        finish_active_goal(WaypointManager::Result::CANCELED,
                                           "canceled by client");
                    }
                    break;
                }

                case rclcpp_action::ResultCode::ABORTED: {
                    spdlog::warn("ReferenceFilter goal aborted unexpectedly");
                    if (wm_canceling) {
                        finish_active_goal(WaypointManager::Result::CANCELED,
                                           "canceled by client");
                    } else {
                        finish_active_goal(
                            WaypointManager::Result::REFERENCE_FILTER_ABORTED,
                            "reference filter aborted waypoint " +
                                std::to_string(current_index_));
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

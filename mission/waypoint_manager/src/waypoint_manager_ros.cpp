#include "waypoint_manager/waypoint_manager_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <spdlog/spdlog.h>
#include <tf2_geometry_msgs.hpp>
#include <functional>
#include <cmath>

namespace vortex::mission {

WaypointManagerNode::WaypointManagerNode(const rclcpp::NodeOptions& options)
: Node("waypoint_manager_node", options)
{
    cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    set_reference_action_client();
    set_waypoint_action_server();
    set_waypoint_service_servers();

    spdlog::info("WaypointManagerNode started");
}

// ---------------------------------------------------------
// Setup functions
// ---------------------------------------------------------

void WaypointManagerNode::set_reference_action_client()
{
    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(
            this,
            "reference_filter_waypoint",
            cb_group_);

    if (!reference_filter_client_->wait_for_action_server(std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilterWaypoint action server not ready");
    }
}

void WaypointManagerNode::set_waypoint_action_server()
{
    waypoint_action_server_ =
        rclcpp_action::create_server<WaypointManager>(
            this,
            "waypoint_manager",
            std::bind(&WaypointManagerNode::handle_waypoint_goal,
                      this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaypointManagerNode::handle_waypoint_cancel,
                      this, std::placeholders::_1),
            std::bind(&WaypointManagerNode::handle_waypoint_accepted,
                      this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_);
}

void WaypointManagerNode::set_waypoint_service_servers()
{
    waypoint_following_service_server_ =
        this->create_service<vortex_msgs::srv::WaypointFollowing>(
            "waypoint_following",
            std::bind(&WaypointManagerNode::handle_waypoint_following_service_request,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

    waypoint_addition_service_server_ =
        this->create_service<vortex_msgs::srv::WaypointAddition>(
            "waypoint_addition",
            std::bind(&WaypointManagerNode::handle_waypoint_addition_service_request,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));
}

// ---------------------------------------------------------
// Waypoint action server callbacks
// ---------------------------------------------------------

rclcpp_action::GoalResponse WaypointManagerNode::handle_waypoint_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const WaypointManager::Goal> goal)
{
    std::lock_guard<std::mutex> lock(mission_mutex_);

    // Abort previous action goal if any
    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        active_action_goal_->abort(res);
    }

    // Action override: reset mission completely
    waypoint_queue_.clear();
    waypoint_queue_.reserve(goal->waypoints.size());
    for (const auto& wp : goal->waypoints) {
        waypoint_queue_.push_back(wp);
    }

    current_index_          = 0;
    switching_threshold_    = goal->switching_threshold;
    persistent_action_mode_ = goal->persistent;
    exec_origin_            = ExecutionOrigin::ACTION;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        active_action_goal_ = goal_handle;
    }

    if (!execution_running_) {
        execution_running_ = true;
        std::thread([this, goal_handle]() {
            this->execute_waypoint_loop(goal_handle);
        }).detach();
    }
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    (void)goal_handle;
    // The execute loop checks is_canceling() and performs the actual stop.
    return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------
// Waypoint services
// ---------------------------------------------------------

// Service 1: Start / overwrite a service-only mission.
// This ALWAYS overrides any existing mission, including persistent actions.
void WaypointManagerNode::handle_waypoint_following_service_request(
    const std::shared_ptr<vortex_msgs::srv::WaypointFollowing::Request> request,
    std::shared_ptr<vortex_msgs::srv::WaypointFollowing::Response> response)
{
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);

        // If an action goal is active, abort it (service overrides everything)
        if (active_action_goal_ && active_action_goal_->is_active()) {
            auto res = std::make_shared<WaypointManager::Result>();
            res->success = false;
            active_action_goal_->abort(res);
            active_action_goal_.reset();
        }

        // Hard override mission
        waypoint_queue_.clear();
        waypoint_queue_.reserve(request->waypoints.size());
        for (const auto& wp : request->waypoints) {
            waypoint_queue_.push_back(wp);
        }

        current_index_          = 0;
        switching_threshold_    = request->switching_threshold;
        persistent_action_mode_ = false; // no action semantics
        exec_origin_            = ExecutionOrigin::SERVICE_ONLY;
    }

    // Start loop if not already running
    if (!execution_running_) {
        execution_running_ = true;
        std::thread([this]() {
            this->execute_waypoint_loop(nullptr);
        }).detach();
    }

    response->success = true;
}

// Service 2: Modify mission (append / overwrite) with blocking rules.
// Only allowed when executing a PERSISTENT ACTION mission.
void WaypointManagerNode::handle_waypoint_addition_service_request(
    const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
    std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response)
{
    std::lock_guard<std::mutex> lock(mission_mutex_);

    // 1. Only allowed when executing a persistent ACTION mission
    if (exec_origin_ != ExecutionOrigin::ACTION || !persistent_action_mode_) {
        response->success = false;
        return;
    }

    // 2. If no mission exists yet (rare but possible), initialize queue
    if (waypoint_queue_.empty()) {
        waypoint_queue_       = request->waypoints;
        current_index_        = 0;
        switching_threshold_  = request->switching_threshold;
        response->success     = true;
        return;
    }

    // 3. BLOCKING RULE:
    // If currently executing a blocking waypoint, all new waypoints must be blocking.
    bool active_blocking = waypoint_queue_[current_index_].blocking;

    if (active_blocking) {
        for (const auto& wp : request->waypoints) {
            if (!wp.blocking) {
                response->success = false;
                return;
            }
        }
    }
    // If active_blocking == false, any new waypoints are allowed.

    // 4. Overwrite or append
    if (request->overwrite) {
        waypoint_queue_      = request->waypoints;
        current_index_       = 0;
        switching_threshold_ = request->switching_threshold;
        response->success    = true;
    } else {
        waypoint_queue_.insert(
            waypoint_queue_.end(),
            request->waypoints.begin(),
            request->waypoints.end());
        switching_threshold_ = request->switching_threshold;
        response->success    = true;
    }
}

// ---------------------------------------------------------
// ReferenceFilter client helpers
// ---------------------------------------------------------

void WaypointManagerNode::send_reference_filter_goal(
    const ReferenceFilterAction::Goal& goal_msg)
{
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        if (active_reference_filter_goal_) {
            reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
        }
    }
    rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
        &WaypointManagerNode::reference_filter_goal_response_callback,
        this,
        std::placeholders::_1);

    options.feedback_callback = std::bind(
        &WaypointManagerNode::reference_filter_feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2);

    options.result_callback = std::bind(
        &WaypointManagerNode::reference_filter_result_callback,
        this,
        std::placeholders::_1);

    reference_filter_client_->async_send_goal(goal_msg, options);
}



void WaypointManagerNode::reference_filter_goal_response_callback(
    std::shared_future<ReferenceFilterGoalHandle::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        spdlog::warn("ReferenceFilterWaypoint goal was rejected");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        active_reference_filter_goal_ = goal_handle;
    }

    spdlog::info("ReferenceFilterWaypoint goal accepted");
}

void WaypointManagerNode::reference_filter_feedback_callback(
    ReferenceFilterGoalHandle::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const ReferenceFilterAction::Feedback> feedback)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = feedback->reference.x;
    pose.position.y = feedback->reference.y;
    pose.position.z = feedback->reference.z;

    tf2::Quaternion q;
    q.setRPY(
        feedback->reference.roll,
        feedback->reference.pitch,
        feedback->reference.yaw);
    pose.orientation = tf2::toMsg(q);

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        current_reference_pose_ = pose;
        have_reference_pose_    = true;
    }
}

void WaypointManagerNode::reference_filter_result_callback(
    const ReferenceFilterGoalHandle::WrappedResult& result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            spdlog::info("ReferenceFilterWaypoint succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            spdlog::warn("ReferenceFilterWaypoint aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            spdlog::info("ReferenceFilterWaypoint canceled");
            break;
        default:
            spdlog::warn("ReferenceFilterWaypoint: unknown result code");
            break;
    }
}

// ---------------------------------------------------------
// Execute loop
// ---------------------------------------------------------

void WaypointManagerNode::execute_waypoint_loop(
    const std::shared_ptr<WaypointManagerGoalHandle> action_goal)
{
    rclcpp::Rate loop_rate(10.0); // 10 Hz

    auto action_result = std::make_shared<WaypointManager::Result>();

    while (rclcpp::ok()) {
        // 1. Check for action cancel
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            if (exec_origin_ == ExecutionOrigin::ACTION &&
                action_goal && action_goal->is_canceling())
            {
                action_result->success = false;
                action_goal->canceled(action_result);
                execution_running_ = false;
                exec_origin_       = ExecutionOrigin::NONE;
                return;
            }
        }

        // 2. Snapshot mission state
        vortex_msgs::msg::Waypoint current_wp;
        geometry_msgs::msg::Pose current_pose;
        bool have_wp       = false;
        bool have_pose     = false;
        bool persistent    = false;
        ExecutionOrigin origin_copy;

        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            origin_copy = exec_origin_;
            persistent  = persistent_action_mode_;

            if (current_index_ < waypoint_queue_.size()) {
                current_wp = waypoint_queue_[current_index_];
                have_wp    = true;
            }

            if (have_reference_pose_) {
                current_pose = current_reference_pose_;
                have_pose    = true;
            }
        }

        // 3. Handle empty queue
        if (!have_wp) {
            if (origin_copy == ExecutionOrigin::ACTION && !persistent) {
                // Non-persistent action: complete when queue is empty
                std::lock_guard<std::mutex> lock(mission_mutex_);
                if (action_goal && action_goal->is_active()) {
                    if (have_reference_pose_) {
                        action_result->final_pose = current_reference_pose_;
                    }
                    action_result->success = true;
                    action_goal->succeed(action_result);
                }
                execution_running_ = false;
                exec_origin_       = ExecutionOrigin::NONE;
                return;
            }

            if (origin_copy == ExecutionOrigin::SERVICE_ONLY) {
                // Service-only mission ends when queue is empty
                execution_running_ = false;
                exec_origin_       = ExecutionOrigin::NONE;
                return;
            }

            // Persistent action with empty queue: wait for more waypoints
            loop_rate.sleep();
            continue;
        }

        // 4. If we don't yet have a pose, wait
        if (!have_pose) {
            loop_rate.sleep();
            continue;
        }

        // 5. Compute distance to current waypoint
        double dx = current_wp.pose.position.x - current_pose.position.x;
        double dy = current_wp.pose.position.y - current_pose.position.y;
        double dz = current_wp.pose.position.z - current_pose.position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 6. Publish feedback if in action mode
        if (origin_copy == ExecutionOrigin::ACTION && action_goal) {
            auto fb = std::make_shared<WaypointManager::Feedback>();
            fb->current_pose     = current_pose;
            fb->current_waypoint = current_wp;
            action_goal->publish_feedback(fb);
        }

        // 7. Check switching condition
        if (dist < switching_threshold_) {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            if (current_index_ < waypoint_queue_.size()) {
                current_index_++;
            }
        }

        loop_rate.sleep();
    }

    // 8. rclcpp::ok() is false – clean up
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        if (exec_origin_ == ExecutionOrigin::ACTION && action_goal && action_goal->is_active()) {
            action_result->success = false;
            action_goal->abort(action_result);
        }
        execution_running_ = false;
        exec_origin_       = ExecutionOrigin::NONE;
    }
}

} // namespace vortex::mission

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::mission::WaypointManagerNode)

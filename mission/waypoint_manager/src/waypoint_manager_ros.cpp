#include "waypoint_manager/waypoint_manager_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <spdlog/spdlog.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <functional>
#include <cmath>

namespace vortex::mission {

WaypointManagerNode::WaypointManagerNode(const rclcpp::NodeOptions& options)
: Node("waypoint_manager_node", options)
{
    cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    set_reference_action_client();
    set_waypoint_action_server();
    set_waypoint_addition_service();

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

void WaypointManagerNode::set_waypoint_addition_service()
{
    waypoint_addition_service_server_ =
        this->create_service<vortex_msgs::srv::WaypointAddition>(
            "waypoint_addition",
            std::bind(&WaypointManagerNode::handle_waypoint_addition_service_request,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2),
                      rmw_qos_profile_services_default,
                    cb_group_);
}

// ---------------------------------------------------------
// Waypoint action server
// ---------------------------------------------------------

rclcpp_action::GoalResponse WaypointManagerNode::handle_waypoint_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const WaypointManager::Goal> goal)
{
    std::lock_guard<std::mutex> lock(mission_mutex_);

    // Abort previous goal if active
    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        active_action_goal_->abort(res);
    }

    // Initialize fresh mission
    waypoint_queue_ = goal->waypoints;
    current_index_  = 0;
    switching_threshold_ = goal->switching_threshold;
    persistent_action_mode_ = goal->persistent;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        active_action_goal_ = goal_handle;
    }

    spdlog::info("WaypointManager: Goal accepted, starting execution timer");

    // Create timer only if not already running
    if (!execution_timer_) {
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),   // 10 Hz
            std::bind(&WaypointManagerNode::execution_step, this),
            cb_group_);
    }

    execution_running_ = true;
}


rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointManagerNode::stop_execution_timer()
{
    execution_running_ = false;

    if (execution_timer_) {
        execution_timer_->cancel();
        execution_timer_.reset();
    }

    std::lock_guard<std::mutex> lock(mission_mutex_);
    active_action_goal_.reset();
}


// ---------------------------------------------------------
// WaypointAddition service
// ---------------------------------------------------------

void WaypointManagerNode::handle_waypoint_addition_service_request(
    const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
    std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response)
{
    std::lock_guard<std::mutex> lock(mission_mutex_);

    // Only allowed when executing a persistent action
    if (!persistent_action_mode_) {
        response->success = false;
        return;
    }

    // Initialize queue if empty
    if (waypoint_queue_.empty()) {
        waypoint_queue_ = request->waypoints;
        current_index_  = 0;
        switching_threshold_ = request->switching_threshold;
        response->success = true;
        return;
    }

    // Blocking rule
    bool active_blocking = waypoint_queue_[current_index_].blocking;
    if (active_blocking) {
        for (const auto& wp : request->waypoints) {
            if (!wp.blocking) {
                response->success = false;
                return;
            }
        }
    }

    // Overwrite or append
    if (request->overwrite) {
        waypoint_queue_ = request->waypoints;
        current_index_  = 0;
    } else {
        waypoint_queue_.insert(
            waypoint_queue_.end(),
            request->waypoints.begin(),
            request->waypoints.end());
    }

    switching_threshold_ = request->switching_threshold;
    response->success = true;
}

// ---------------------------------------------------------
// ReferenceFilter client
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
    options.goal_response_callback =
        [this](ReferenceFilterGoalHandle::SharedPtr goal_handle)
        {
            reference_filter_goal_response_callback(goal_handle);
        };

    options.feedback_callback =
        [this](ReferenceFilterGoalHandle::SharedPtr,
               const std::shared_ptr<const ReferenceFilterAction::Feedback> feedback)
        {
            reference_filter_feedback_callback(nullptr, feedback);
        };

    options.result_callback =
        [this](const ReferenceFilterGoalHandle::WrappedResult & result)
        {
            reference_filter_result_callback(result);
        };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

void WaypointManagerNode::reference_filter_goal_response_callback(
    ReferenceFilterGoalHandle::SharedPtr goal_handle)
{
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
    ReferenceFilterGoalHandle::SharedPtr,
    const std::shared_ptr<const ReferenceFilterAction::Feedback> feedback)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = feedback->reference.x;
    pose.position.y = feedback->reference.y;
    pose.position.z = feedback->reference.z;

    tf2::Quaternion q;
    q.setRPY(feedback->reference.roll,
             feedback->reference.pitch,
             feedback->reference.yaw);

    pose.orientation = tf2::toMsg(q);

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        current_reference_pose_ = pose;
        have_reference_pose_ = true;
    }
}

void WaypointManagerNode::reference_filter_result_callback(
    const ReferenceFilterGoalHandle::WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::ABORTED) {
        spdlog::warn("ReferenceFilterWaypoint aborted");
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        spdlog::info("ReferenceFilterWaypoint canceled");
    }
}

// ---------------------------------------------------------
// Execute loop
// ---------------------------------------------------------

void WaypointManagerNode::execution_step()
{
    std::shared_ptr<WaypointManagerGoalHandle> goal_handle;
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        goal_handle = active_action_goal_;
    }

    if (!goal_handle) {
        return;  // no active mission → timer does nothing
    }

    auto action_result = std::make_shared<WaypointManager::Result>();

    // --- Cancel requested ---
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        if (goal_handle->is_canceling()) {
            action_result->success = false;
            goal_handle->canceled(action_result);
            stop_execution_timer();
            return;
        }
    }

    // --- Snapshot mission state ---
    vortex_msgs::msg::Waypoint current_wp;
    geometry_msgs::msg::Pose current_pose;
    bool have_wp = false;
    bool have_pose = false;
    bool persistent = false;

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        persistent = persistent_action_mode_;

        if (current_index_ < waypoint_queue_.size()) {
            current_wp = waypoint_queue_[current_index_];
            have_wp = true;
        }

        if (have_reference_pose_) {
            current_pose = current_reference_pose_;
            have_pose = true;
        }
    }

    // --- No waypoint available ---
    if (!have_wp) {
        if (!persistent) {
            std::lock_guard<std::mutex> lock(mission_mutex_);

            if (goal_handle->is_active()) {
                if (have_reference_pose_) {
                    action_result->final_pose = current_reference_pose_;
                }
                action_result->success = true;
                goal_handle->succeed(action_result);
            }

            stop_execution_timer();
            return;
        }

        return;  // persistent mode → wait for more waypoints
    }

    if (!have_pose) {
        return; // wait for reference filter data
    }

    // --- Compute distance to waypoint ---
    double dx = current_wp.pose.position.x - current_pose.position.x;
    double dy = current_wp.pose.position.y - current_pose.position.y;
    double dz = current_wp.pose.position.z - current_pose.position.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    // --- Publish feedback ---
    auto fb = std::make_shared<WaypointManager::Feedback>();
    fb->current_pose     = current_pose;
    fb->current_waypoint = current_wp;
    goal_handle->publish_feedback(fb);

    // --- Switch waypoint? ---
    if (dist < switching_threshold_) {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        current_index_++;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointManagerNode)

} // namespace vortex::mission

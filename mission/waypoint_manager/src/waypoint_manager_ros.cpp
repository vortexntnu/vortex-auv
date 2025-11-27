#include "waypoint_manager/waypoint_manager_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <spdlog/spdlog.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace vortex::mission {

WaypointManagerNode::WaypointManagerNode(const rclcpp::NodeOptions &options)
: Node("waypoint_manager_node", options)
{
    set_reference_action_client();
    set_waypoint_action_server();
    set_waypoint_addition_service();

    spdlog::info("WaypointManagerNode started");
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------

void WaypointManagerNode::set_reference_action_client()
{
    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(
            this, "reference_filter_waypoint");

    if (!reference_filter_client_->wait_for_action_server(std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilter server not ready");
    }
}

void WaypointManagerNode::set_waypoint_action_server()
{
    waypoint_action_server_ =
        rclcpp_action::create_server<WaypointManager>(
            this,
            "waypoint_manager",
            std::bind(&WaypointManagerNode::handle_waypoint_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaypointManagerNode::handle_waypoint_cancel, this,
                      std::placeholders::_1),
            std::bind(&WaypointManagerNode::handle_waypoint_accepted, this,
                      std::placeholders::_1));
}

void WaypointManagerNode::set_waypoint_addition_service()
{
    waypoint_addition_service_server_ =
        this->create_service<vortex_msgs::srv::WaypointAddition>(
            "waypoint_addition",
            std::bind(&WaypointManagerNode::handle_waypoint_addition_service_request,
                      this, std::placeholders::_1, std::placeholders::_2));
}

// ---------------------------------------------------------
// ACTION GOAL
// ---------------------------------------------------------

rclcpp_action::GoalResponse WaypointManagerNode::handle_waypoint_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WaypointManager::Goal> goal)
{
    // Cancel existing mission
    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        active_action_goal_->abort(res);
    }

    waypoint_queue_ = goal->waypoints;
    current_index_ = 0;
    persistent_action_mode_ = goal->persistent;
    switching_threshold_ = goal->switching_threshold;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    active_action_goal_ = goal_handle;

    // Send first RF goal immediately
    if (!waypoint_queue_.empty()) {
        ReferenceFilterAction::Goal rf_goal;
        rf_goal.waypoint = waypoint_queue_[0];
        send_reference_filter_goal(rf_goal);
    }

    if (!execution_timer_) {
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WaypointManagerNode::execution_step, this));
    }

    execution_running_ = true;
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointManagerNode::stop_execution_timer()
{
    execution_running_ = false;

    if (execution_timer_) {
        execution_timer_->cancel();
        execution_timer_.reset();
    }

    active_action_goal_.reset();
}

// ---------------------------------------------------------
// WAYPOINT ADDITION SERVICE
// ---------------------------------------------------------

void WaypointManagerNode::handle_waypoint_addition_service_request(
    const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
    std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response)
{
    if (!persistent_action_mode_) {
        response->success = false;
        return;
    }

    if (waypoint_queue_.empty()) {
        waypoint_queue_ = request->waypoints;
        current_index_ = 0;
        switching_threshold_ = request->switching_threshold;
        response->success = true;
        return;
    }

    bool active_blocking = waypoint_queue_[current_index_].blocking;

    if (active_blocking) {
        for (auto &wp : request->waypoints) {
            if (!wp.blocking) {
                response->success = false;
                return;
            }
        }
    }

    if (request->overwrite) {
        waypoint_queue_ = request->waypoints;
        current_index_ = 0;
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
// REFERENCE FILTER CLIENT
// ---------------------------------------------------------

void WaypointManagerNode::send_reference_filter_goal(
    const ReferenceFilterAction::Goal &goal_msg)
{
    // Cancel previous goal if one is active
    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions options;

    options.goal_response_callback =
        [this](ReferenceFilterGoalHandle::SharedPtr gh)
        {
            if (!gh) {
                spdlog::warn("ReferenceFilter goal rejected");
                return;
            }

            active_reference_filter_goal_ = gh;
            spdlog::info("ReferenceFilter goal accepted");
        };

    options.feedback_callback =
        [this](
            ReferenceFilterGoalHandle::SharedPtr,
            const std::shared_ptr<const ReferenceFilterAction::Feedback> fb)
        {
            // Store *only the raw numbers* (fast, cheap)
            latest_ref_feedback_ = *fb;
            have_reference_pose_ = true;
        };

    options.result_callback =
        [this](const ReferenceFilterGoalHandle::WrappedResult &res)
        {
            if (res.code == rclcpp_action::ResultCode::ABORTED) {
                spdlog::warn("ReferenceFilter goal aborted unexpectedly");
            }
            // Reference filter goal no longer active
            active_reference_filter_goal_.reset();
            // maybe reset ref and has_pose here?
        };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

// ---------------------------------------------------------
// EXECUTION LOOP
// ---------------------------------------------------------

void WaypointManagerNode::execution_step()
{
    auto goal_handle = active_action_goal_;

    if (!goal_handle)
        return;

    // Check cancel
    if (goal_handle->is_canceling()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        goal_handle->canceled(res);
        stop_execution_timer();
        return;
    }

    // Check waypoint availability
    if (current_index_ >= waypoint_queue_.size()) {

        if (!persistent_action_mode_) {
            auto res = std::make_shared<WaypointManager::Result>();
            res->success = true;

            if (have_reference_pose_) {
                geometry_msgs::msg::Pose final_pose;
                final_pose.position.x = latest_ref_feedback_.reference.x;
                final_pose.position.y = latest_ref_feedback_.reference.y;
                final_pose.position.z = latest_ref_feedback_.reference.z;

                tf2::Quaternion q;
                q.setRPY(latest_ref_feedback_.reference.roll,
                         latest_ref_feedback_.reference.pitch,
                         latest_ref_feedback_.reference.yaw);
                final_pose.orientation = tf2::toMsg(q);

                res->final_pose = final_pose;
            }

            goal_handle->succeed(res);
            stop_execution_timer();
        }

        return;  // persistent mode: wait for more
    }

    if (!have_reference_pose_)
        return;

    // Build robot pose
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = latest_ref_feedback_.reference.x;
    robot_pose.position.y = latest_ref_feedback_.reference.y;
    robot_pose.position.z = latest_ref_feedback_.reference.z;

    tf2::Quaternion q;
    q.setRPY(latest_ref_feedback_.reference.roll,
             latest_ref_feedback_.reference.pitch,
             latest_ref_feedback_.reference.yaw);

    robot_pose.orientation = tf2::toMsg(q);

    // Current waypoint
    const auto &wp = waypoint_queue_[current_index_];

    // Distance
    double dx = wp.pose.position.x - robot_pose.position.x;
    double dy = wp.pose.position.y - robot_pose.position.y;
    double dz = wp.pose.position.z - robot_pose.position.z;

    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Send feedback
    auto fb = std::make_shared<WaypointManager::Feedback>();
    fb->current_pose = robot_pose;
    fb->current_waypoint = wp;
    goal_handle->publish_feedback(fb);

    // Switch
    if (dist < switching_threshold_) {

        current_index_++;

        if (current_index_ < waypoint_queue_.size()) {
            ReferenceFilterAction::Goal rf_goal;
            rf_goal.waypoint = waypoint_queue_[current_index_];
            send_reference_filter_goal(rf_goal);
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointManagerNode)

} // namespace vortex::mission

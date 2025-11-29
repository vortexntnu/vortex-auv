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
    set_waypoint_service_server();

    spdlog::info("WaypointManagerNode started");
}

// ---------------------------------------------------------
// SETUP INTERFACES
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

void WaypointManagerNode::set_waypoint_service_server()
{
    waypoint_service_server_ =
        this->create_service<vortex_msgs::srv::WaypointAddition>(
            "waypoint_addition",
            std::bind(&WaypointManagerNode::handle_waypoint_addition_service_request,
                      this, std::placeholders::_1, std::placeholders::_2));
}

// ---------------------------------------------------------
// WAYPOINT MANAGER ACTION SERVER
// ---------------------------------------------------------

rclcpp_action::GoalResponse WaypointManagerNode::handle_waypoint_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WaypointManager::Goal> goal)
{
    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        active_action_goal_->abort(res);
    }

    waypoints_ = goal->waypoints;
    current_index_ = 0;
    persistent_action_mode_ = goal->persistent;
    convergence_threshold_ = goal->convergence_threshold;
    non_interruptible_mode_ = false;
    have_reference_pose_ = false;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    active_action_goal_ = goal_handle;

    if (!waypoints_.empty()) {
        ReferenceFilterAction::Goal rf_goal;
        rf_goal.waypoint = waypoints_[0];
        rf_goal.convergence_threshold = convergence_threshold_;
        send_reference_filter_goal(rf_goal);
    }

    if (!execution_timer_) {
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&WaypointManagerNode::execution_step, this));
    }
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------
// WAYPOINT MANAGER SERVICE SERVER
// ---------------------------------------------------------

void WaypointManagerNode::handle_waypoint_addition_service_request(
    const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
    std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response)
{
    if (!persistent_action_mode_) {
        response->success = false;
        return;
    }
    
    if (waypoints_.empty()) {
        waypoints_ = request->waypoints;
        current_index_ = 0;
        have_reference_pose_ = false;
        non_interruptible_mode_ = request->non_interruptible;
        response->success = true;
        return;
    }

    if (non_interruptible_mode_) {
        if (!request->non_interruptible) {
            response->success = false;
            return;
        }
    }

    non_interruptible_mode_ = request->non_interruptible;

    if (request->overwrite) {
        waypoints_ = request->waypoints;
        current_index_ = 0;
        have_reference_pose_ = false;
    } else {
        waypoints_.insert(
            waypoints_.end(),
            request->waypoints.begin(),
            request->waypoints.end());
    }

    response->success = true;
}

// ---------------------------------------------------------
// REFERENCE FILTER ACTION CLIENT
// ---------------------------------------------------------

void WaypointManagerNode::send_reference_filter_goal(
    const ReferenceFilterAction::Goal &goal_msg)
{
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
        };

    options.feedback_callback =
        [this](
            ReferenceFilterGoalHandle::SharedPtr,
            const std::shared_ptr<const ReferenceFilterAction::Feedback> fb)
        {
            latest_ref_feedback_ = *fb;
            have_reference_pose_ = true;
        };

    options.result_callback =
    [this](const ReferenceFilterGoalHandle::WrappedResult &res)
    {
        active_reference_filter_goal_.reset();
        have_reference_pose_ = false;

        if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
            spdlog::info("ReferenceFilter goal reached waypoint");
            on_reference_filter_succeeded();
            return;
        } else {
            spdlog::warn("ReferenceFilter goal failed with code: {}", static_cast<int>(res.code));
            return;
        }
        
    };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

void WaypointManagerNode::on_reference_filter_succeeded()
{
    if (current_index_ < waypoints_.size()) {
        current_index_++;

        if (current_index_ < waypoints_.size()) {
            ReferenceFilterAction::Goal rf_goal;
            rf_goal.waypoint = waypoints_[current_index_];
            rf_goal.convergence_threshold = convergence_threshold_;
            send_reference_filter_goal(rf_goal);
        }
    }
}

// ---------------------------------------------------------
// EXECUTION LOOP
// ---------------------------------------------------------


void WaypointManagerNode::stop_execution_timer()
{
    if (execution_timer_) {
        execution_timer_->cancel();
        execution_timer_.reset();
    }

    active_action_goal_.reset();
}

void WaypointManagerNode::execution_step()
{
    auto goal_handle = active_action_goal_;

    if (!goal_handle)
        return;

    if (goal_handle->is_canceling()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        res->pose_valid = have_reference_pose_;
        goal_handle->canceled(res);
        stop_execution_timer();
        return;
    }

    if (current_index_ >= waypoints_.size()) {

        if (!persistent_action_mode_) {
            auto res = std::make_shared<WaypointManager::Result>();
            res->success = true;

            if (have_reference_pose_) {
                res->final_pose = reference_to_pose(latest_ref_feedback_);
            }
            res->pose_valid = have_reference_pose_;

            goal_handle->succeed(res);
            stop_execution_timer();
        }

        return;  // persistent mode: wait for more
    }

    if (!have_reference_pose_)
        return;

    geometry_msgs::msg::Pose robot_pose =
        reference_to_pose(latest_ref_feedback_);

    auto fb = std::make_shared<WaypointManager::Feedback>();
    fb->current_pose = robot_pose;
    fb->current_waypoint = waypoints_[current_index_];
    goal_handle->publish_feedback(fb);

}

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointManagerNode)

} // namespace vortex::mission

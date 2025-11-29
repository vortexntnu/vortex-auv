#include "waypoint_manager/waypoint_manager_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <spdlog/spdlog.h>
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
// HELPERS
// ---------------------------------------------------------

geometry_msgs::msg::Pose
WaypointManagerNode::reference_to_pose(
    const ReferenceFilterAction::Feedback &fb) const
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = fb.reference.x;
    pose.position.y = fb.reference.y;
    pose.position.z = fb.reference.z;

    tf2::Quaternion q;
    q.setRPY(fb.reference.roll,
             fb.reference.pitch,
             fb.reference.yaw);

    pose.orientation = tf2::toMsg(q);
    return pose;
}

void WaypointManagerNode::cleanup_mission_state()
{
    waypoints_.clear();
    current_index_ = 0;
    persistent_action_mode_ = false;
    non_interruptible_mode_ = false;
    have_reference_pose_ = false;

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    active_action_goal_.reset();
}

void WaypointManagerNode::send_next_reference_filter_goal()
{
    if (current_index_ >= waypoints_.size()) {
        if (!persistent_action_mode_ && active_action_goal_ && active_action_goal_->is_active()) {
            auto res = std::make_shared<WaypointManager::Result>();
            res->success = true;
            res->pose_valid = have_reference_pose_;
            if (have_reference_pose_) {
                res->final_pose = reference_to_pose(latest_ref_feedback_);
            }
            active_action_goal_->succeed(res);
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
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WaypointManager::Goal> goal)
{
    if (active_action_goal_ && active_action_goal_->is_active()) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = false;
        res->pose_valid = have_reference_pose_;
        if (have_reference_pose_) {
            res->final_pose = reference_to_pose(latest_ref_feedback_);
        }
        active_action_goal_->abort(res);
    }

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    waypoints_ = goal->waypoints;
    current_index_ = 0;
    persistent_action_mode_ = goal->persistent;
    non_interruptible_mode_ = false;
    have_reference_pose_ = false;
    convergence_threshold_ = goal->convergence_threshold;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void WaypointManagerNode::handle_waypoint_accepted(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    spdlog::info("WaypointManager: action goal accepted");
    active_action_goal_ = goal_handle;

    if (!waypoints_.empty()) {
        send_next_reference_filter_goal();
    } else if (!persistent_action_mode_) {
        auto res = std::make_shared<WaypointManager::Result>();
        res->success = true;
        res->pose_valid = false;
        goal_handle->succeed(res);
        cleanup_mission_state();
    }
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_waypoint_cancel(
    const std::shared_ptr<WaypointManagerGoalHandle> goal_handle)
{
    spdlog::info("WaypointManager: cancel requested");

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    auto res = std::make_shared<WaypointManager::Result>();
    res->success = false;
    res->pose_valid = have_reference_pose_;
    if (have_reference_pose_) {
        res->final_pose = reference_to_pose(latest_ref_feedback_);
    }

    goal_handle->canceled(res);
    cleanup_mission_state();

    return rclcpp_action::CancelResponse::ACCEPT;
}

// ---------------------------------------------------------
// WAYPOINT MANAGER SERVICE SERVER
// ---------------------------------------------------------

void WaypointManagerNode::handle_waypoint_addition_service_request(
    const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
    std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response)
{
    if (!persistent_action_mode_ ||
        !active_action_goal_ ||
        !active_action_goal_->is_active())
    {
        response->success = false;
        return;
    }

    if (non_interruptible_mode_ && !request->non_interruptible) {
        response->success = false;
        return;
    }

    non_interruptible_mode_ = request->non_interruptible;

    if (request->overwrite) {

        waypoints_ = request->waypoints;
        current_index_ = 0;
        have_reference_pose_ = false;

        if (active_reference_filter_goal_) {
            reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
            active_reference_filter_goal_.reset();
        }

        if (!waypoints_.empty()) {
            send_next_reference_filter_goal();
        }

        response->success = true;
        return;
    }

    waypoints_.insert(
        waypoints_.end(),
        request->waypoints.begin(),
        request->waypoints.end());

    if (!active_reference_filter_goal_ &&
        current_index_ < waypoints_.size())
    {
        send_next_reference_filter_goal();
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

            if (!active_action_goal_ || !active_action_goal_->is_active())
                return;

            geometry_msgs::msg::Pose robot_pose =
                reference_to_pose(*fb);

            if (current_index_ < waypoints_.size()) {
                auto wm_fb = std::make_shared<WaypointManager::Feedback>();
                wm_fb->current_pose = robot_pose;
                wm_fb->current_waypoint = waypoints_[current_index_];
                active_action_goal_->publish_feedback(wm_fb);
            }
        };

    options.result_callback =
        [this](const ReferenceFilterGoalHandle::WrappedResult &res)
        {
            active_reference_filter_goal_.reset();

            switch (res.code) {

                case rclcpp_action::ResultCode::SUCCEEDED:
                    spdlog::info("ReferenceFilter goal reached waypoint");
                    on_reference_filter_succeeded();
                    break;

                case rclcpp_action::ResultCode::CANCELED:
                    spdlog::info("ReferenceFilter goal cancelled");
                    break;

                case rclcpp_action::ResultCode::ABORTED:
                    spdlog::warn("ReferenceFilter goal aborted unexpectedly");
                    if (active_action_goal_ && active_action_goal_->is_active()) {
                        auto res_msg = std::make_shared<WaypointManager::Result>();
                        res_msg->success = false;
                        res_msg->pose_valid = have_reference_pose_;
                        if (have_reference_pose_) {
                            res_msg->final_pose = reference_to_pose(latest_ref_feedback_);
                        }
                        active_action_goal_->abort(res_msg);
                        cleanup_mission_state();
                    }
                    break;
                
                default:
                    spdlog::error("ReferenceFilter goal returned unknown result code");
                    break;
            }
        };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

void WaypointManagerNode::on_reference_filter_succeeded()
{
    if (!active_action_goal_ || !active_action_goal_->is_active()) {
        return;
    }

    current_index_++;

    if (current_index_ >= waypoints_.size()) {

        if (!persistent_action_mode_) {
            auto res = std::make_shared<WaypointManager::Result>();
            res->success = true;
            res->pose_valid = have_reference_pose_;
            if (have_reference_pose_) {
                res->final_pose = reference_to_pose(latest_ref_feedback_);
            }
            active_action_goal_->succeed(res);
            cleanup_mission_state();
        }
        return;
    }

    send_next_reference_filter_goal();
}

RCLCPP_COMPONENTS_REGISTER_NODE(WaypointManagerNode)

} // namespace vortex::mission

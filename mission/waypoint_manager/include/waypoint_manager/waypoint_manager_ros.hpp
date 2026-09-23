#ifndef WAYPOINT_MANAGER__WAYPOINT_MANAGER_ROS_HPP_
#define WAYPOINT_MANAGER__WAYPOINT_MANAGER_ROS_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/action/guidance_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

enum class DebugPublishMode { none, timer, on_new_waypoint };

namespace vortex::mission {

using WaypointManager = vortex_msgs::action::WaypointManager;
using WaypointManagerGoalHandle =
    rclcpp_action::ServerGoalHandle<WaypointManager>;

using ReferenceFilterAction = vortex_msgs::action::GuidanceWaypoint;
using ReferenceFilterGoalHandle =
    rclcpp_action::ClientGoalHandle<ReferenceFilterAction>;

class WaypointManagerNode : public rclcpp::Node {
   public:
    explicit WaypointManagerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~WaypointManagerNode() override;

   private:
    // @brief Create the action server for WaypointManager.
    void set_waypoint_action_server();

    // @brief Subscribe to the vehicle pose (needed to resolve relative goals).
    void set_pose_subscription();

    // @brief Create the action client for ReferenceFilterWaypoint.
    void set_reference_action_client();

    // @brief Create the service servers for SendWaypoints.
    void set_waypoint_service_server();

    // @brief Subscribe to the system-wide reset topic.
    void setup_reset_subscription();

    // @brief Abort any active goal and clear state on reset.
    void on_system_reset(std_msgs::msg::Empty::ConstSharedPtr msg);

    // @brief Create the debug waypoint publisher and optional timer.
    void setup_debug_publisher();

    // @brief Publish the current waypoint on the debug topic (no-op if no
    // active waypoint).
    void publish_current_waypoint();

    // @brief Construct the result message for the WaypointManager action
    // @param outcome One of WaypointManager::Result::{SUCCEEDED, PREEMPTED,
    // CANCELED, INVALID_GOAL, REFERENCE_FILTER_ABORTED}
    // @param message Human readable explanation
    // @return The constructed result message
    std::shared_ptr<vortex_msgs::action::WaypointManager_Result>
    construct_result(uint8_t outcome, const std::string& message) const;

    // @brief Terminate the active action goal with the given outcome
    // (succeed/canceled/abort as appropriate) and clean up mission state.
    void finish_active_goal(uint8_t outcome, const std::string& message);

    // @brief Check a goal and resolve relative frames to absolute odom poses.
    // @param goal The incoming goal
    // @param resolved Receives the waypoints in odom
    // @return Empty string if valid, otherwise a description of the problem
    std::string validate_and_resolve_goal(
        const vortex_msgs::action::WaypointManager::Goal& goal,
        std::vector<vortex_msgs::msg::Waypoint>& resolved) const;

    // @brief Clean up the mission state after completion or cancellation of a
    // waypoint action. Cancel active goals and reset internal variables. Make
    // system ready for next action.
    void cleanup_mission_state();

    // @brief Send the next goal to the ReferenceFilter action server based on
    // the current waypoint index or finish the waypoint action if all waypoints
    // have been processed.
    void send_next_reference_filter_goal();

    // @brief Handle incoming action goal requests
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_waypoint_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::WaypointManager::Goal>
            goal_msg);

    // @brief Handle requests to cancel the waypoint action
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_waypoint_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::WaypointManager>> goal_handle);

    // @brief Handle the accepted goal request
    // @param goal_handle The goal handle
    void handle_waypoint_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::WaypointManager>> goal_handle);

    // @brief Handle incoming send waypoints service requests
    //        Only accepted if waypoint action is running.
    // @param request Incoming service request containing waypoint information.
    // @param response Service response that should be populated and sent back
    // to the caller.
    void handle_send_waypoints_service_request(
        const std::shared_ptr<vortex_msgs::srv::SendWaypoints::Request> request,
        std::shared_ptr<vortex_msgs::srv::SendWaypoints::Response> response);

    // @brief Send a goal to the reference filter
    // @param goal_msg The action goal
    void send_reference_filter_goal(
        const vortex_msgs::action::GuidanceWaypoint::Goal& goal_msg);

    rclcpp_action::Client<vortex_msgs::action::GuidanceWaypoint>::SharedPtr
        reference_filter_client_;
    rclcpp_action::Server<vortex_msgs::action::WaypointManager>::SharedPtr
        waypoint_action_server_;
    rclcpp::Service<vortex_msgs::srv::SendWaypoints>::SharedPtr
        waypoint_service_server_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    mutable std::mutex pose_mutex_;
    std::optional<vortex::utils::types::Pose> current_pose_;

    rclcpp::TimerBase::SharedPtr cancel_timer_;

    rclcpp::Publisher<vortex_msgs::msg::Waypoint>::SharedPtr
        debug_waypoint_pub_;
    rclcpp::TimerBase::SharedPtr debug_timer_;
    DebugPublishMode debug_mode_{DebugPublishMode::none};

    std::vector<vortex_msgs::msg::Waypoint> waypoints_{};
    std::size_t current_index_{0};
    double convergence_threshold_{0.1};

    bool persistent_action_mode_active_{false};
    bool priority_mode_active_{false};

    bool is_cancel_in_progress_{false};

    std::uint64_t mission_id_ = 0;

    std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;
    std::shared_ptr<WaypointManagerGoalHandle> active_action_goal_;
};

}  // namespace vortex::mission

#endif  // WAYPOINT_MANAGER__WAYPOINT_MANAGER_ROS_HPP_

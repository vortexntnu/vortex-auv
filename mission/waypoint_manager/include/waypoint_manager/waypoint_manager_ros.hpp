#ifndef WAYPOINT_MANAGER__WAYPOINT_MANAGER_ROS_HPP_
#define WAYPOINT_MANAGER__WAYPOINT_MANAGER_ROS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

namespace vortex::mission {

using WaypointManager = vortex_msgs::action::WaypointManager;
using WaypointManagerGoalHandle =
    rclcpp_action::ServerGoalHandle<WaypointManager>;

using ReferenceFilterAction = vortex_msgs::action::ReferenceFilterWaypoint;
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

    // @brief Create the action client for ReferenceFilterWaypoint.
    void set_reference_action_client();

    // @brief Create the service servers for SendWaypoints.
    void set_waypoint_service_server();

    // @brief Construct the result message for the WaypointManager action
    // @param success Whether the action was successful
    // @return The constructed result message
    std::shared_ptr<vortex_msgs::action::WaypointManager_Result>
    construct_result(bool success) const;

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
        const vortex_msgs::action::ReferenceFilterWaypoint::Goal& goal_msg);

    rclcpp_action::Client<vortex_msgs::action::ReferenceFilterWaypoint>::
        SharedPtr reference_filter_client_;
    rclcpp_action::Server<vortex_msgs::action::WaypointManager>::SharedPtr
        waypoint_action_server_;
    rclcpp::Service<vortex_msgs::srv::SendWaypoints>::SharedPtr
        waypoint_service_server_;

    std::vector<vortex_msgs::msg::Waypoint> waypoints_{};
    std::size_t current_index_{0};
    double convergence_threshold_{0.1};

    bool persistent_action_mode_active_{false};
    bool priority_mode_active_{false};

    ReferenceFilterAction::Feedback latest_ref_feedback_;
    bool has_reference_pose_{false};
    bool is_cancel_in_progress_{false};

    std::uint64_t mission_id_ = 0;

    std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;
    std::shared_ptr<WaypointManagerGoalHandle> active_action_goal_;
};

}  // namespace vortex::mission

#endif  // WAYPOINT_MANAGER__WAYPOINT_MANAGER_ROS_HPP_

#ifndef WAYPOINT_MANAGER_ROS_HPP
#define WAYPOINT_MANAGER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/srv/waypoint.hpp>
#include <vortex_msgs/msg/waypoint.hpp>


namespace vortex::mission {

class WaypointManagerNode : public rclcpp::Node {
    public:
        explicit WaypointManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:

        // @brief Handle incoming action goal requests
        // @param uuid The goal UUID
        // @param goal The goal message
        // @return The goal response
        rclcpp_action::GoalResponse handle_waypoint_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<
                const vortex_msgs::action::WaypointManager::Goal> goal_msg);

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

        // @brief Execute the waypoint action goal
        // @param goal_handle The goal handle
        void execute_waypoint_action(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                vortex_msgs::action::WaypointManager>> goal_handle);

        // @brief Send a goal to the reference filter
        // @param goal_msg The action goal
        void send_reference_filter_goal(
            const vortex_msgs::action::ReferenceFilterWaypoint::Goal & goal_msg);

        // @brief Callback: goal accepted or rejected
        // @param future A future containing the goal handle if the action server accepted the goal,
        //        or nullptr if it was rejected.
        void reference_filter_goal_response_callback(
            std::shared_future<
                rclcpp_action::ClientGoalHandle<
                    vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr> future);

        // @brief Callback: feedback during execution
        // @param goal_handle The handle representing the active action goal sent to the server.
        // @param feedback The feedback message provided by the action server.
        void reference_filter_feedback_callback(
            rclcpp_action::ClientGoalHandle<
                vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr goal_handle,
            const std::shared_ptr<
                const vortex_msgs::action::ReferenceFilterWaypoint::Feedback> feedback);

        // @brief Callback: result when action completes
        // @param result A wrapped result structure returned by the action server,
        //        containing status (succeeded/aborted/canceled) and the result message.
        void reference_filter_result_callback(
            const rclcpp_action::ClientGoalHandle<
                vortex_msgs::action::ReferenceFilterWaypoint>::WrappedResult & result);

        // @brief Handle incoming waypoint service requests
        // @param request Incoming service request containing waypoint information.
        // @param response Service response that should be populated and sent back to the caller.
        void handle_waypoint_service(
            const std::shared_ptr<vortex_msgs::srv::Waypoint::Request> request,
            std::shared_ptr<vortex_msgs::srv::Waypoint::Response> response);


        rclcpp_action::Client<vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr reference_filter_client_;
        rclcpp_action::Server<vortex_msgs::action::WaypointManager>::SharedPtr waypoint_action_server_;
        rclcpp::Service<vortex_msgs::srv::Waypoint>::SharedPtr waypoint_service_server_;

};

} // namespace vortex::mission

#endif // WAYPOINT_MANAGER_ROS_HPP
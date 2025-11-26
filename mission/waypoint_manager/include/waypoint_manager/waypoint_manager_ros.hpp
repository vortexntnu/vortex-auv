#ifndef WAYPOINT_MANAGER_ROS_HPP
#define WAYPOINT_MANAGER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/srv/waypoint_following.hpp>
#include <vortex_msgs/srv/waypoint_addition.hpp>
#include <vortex_msgs/msg/waypoint.hpp>


namespace vortex::mission {

using WaypointManager           = vortex_msgs::action::WaypointManager;
using WaypointManagerGoalHandle = rclcpp_action::ServerGoalHandle<WaypointManager>;
using ReferenceFilterAction     = vortex_msgs::action::ReferenceFilterWaypoint;
using ReferenceFilterGoalHandle = rclcpp_action::ClientGoalHandle<ReferenceFilterAction>;

class WaypointManagerNode : public rclcpp::Node {
    public:
        explicit WaypointManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:

        // @brief Create the action server to handle incoming waypoint goals.
        void set_waypoint_action_server();

        // @brief Create the action client for a reference filter waypoint.
        void set_reference_action_client();

        // @brief Create the service servers to handle incoming waypoint requests.
        void set_waypoint_service_servers();

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

        // @brief Send a goal to the reference filter
        // @param goal_msg The action goal
        void send_reference_filter_goal(
            const vortex_msgs::action::ReferenceFilterWaypoint::Goal & goal_msg);

        // @brief Callback: goal accepted or rejected
        // @param future A future containing the goal handle if the action server accepted the goal.
        void reference_filter_goal_response_callback(
            std::shared_future<
                ReferenceFilterGoalHandle::SharedPtr> future);

        // @brief Callback: feedback during execution
        // @param goal_handle The handle representing the active action goal sent to the server.
        // @param feedback The feedback message provided by the action server.
        void reference_filter_feedback_callback(
            ReferenceFilterGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<
                const vortex_msgs::action::ReferenceFilterWaypoint::Feedback> feedback);

        // @brief Callback: result when action completes
        // @param result A wrapped result structure returned by the action server,
        //        containing status (succeeded/aborted/canceled) and the result message.
        void reference_filter_result_callback(
            const ReferenceFilterGoalHandle::WrappedResult & result);

        // @brief Handle incoming waypoint following requests.
        //        Only accepted if waypoint action is not running. 
        //        Overwrites current waypoints stored.
        // @param request Incoming service request containing waypoint information.
        // @param response Service response that should be populated and sent back to the caller.
        void handle_waypoint_following_service_request(
            const std::shared_ptr<vortex_msgs::srv::WaypointFollowing::Request> request,
            std::shared_ptr<vortex_msgs::srv::WaypointFollowing::Response> response);

        // @brief Handle incoming waypoint addition service requests
        //        Only accepted if waypoint action is running.
        // @param request Incoming service request containing waypoint information.
        // @param response Service response that should be populated and sent back to the caller.
        void handle_waypoint_addition_service_request(
            const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
            std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response);

        void execute_waypoint_loop(
            const std::shared_ptr<WaypointManagerGoalHandle> action_goal);


        rclcpp_action::Client<vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr reference_filter_client_;
        rclcpp_action::Server<vortex_msgs::action::WaypointManager>::SharedPtr waypoint_action_server_;
        rclcpp::Service<vortex_msgs::srv::WaypointFollowing>::SharedPtr waypoint_following_service_server_;
        rclcpp::Service<vortex_msgs::srv::WaypointAddition>::SharedPtr waypoint_addition_service_server_;


        enum class ExecutionOrigin {
            NONE,
            ACTION,
            SERVICE_ONLY
        };

        ExecutionOrigin exec_origin_{ExecutionOrigin::NONE};

        std::vector<vortex_msgs::msg::Waypoint> waypoint_queue_;
        std::size_t current_index_{0};
        double switching_threshold_{1.0};

        bool execution_running_{false};
        bool persistent_action_mode_{false};

        bool have_reference_pose_{false};
        geometry_msgs::msg::Pose current_reference_pose_;

        std::mutex mission_mutex_;


        std::shared_ptr<WaypointManagerGoalHandle> active_action_goal_;
        std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;

        rclcpp::CallbackGroup::SharedPtr cb_group_;


};

} // namespace vortex::mission

#endif // WAYPOINT_MANAGER_ROS_HPP
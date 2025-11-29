#ifndef WAYPOINT_MANAGER_ROS_HPP
#define WAYPOINT_MANAGER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
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

        // @brief Create the action server for WaypointManager.
        void set_waypoint_action_server();

        // @brief Create the action client for ReferenceFilterWaypoint.
        void set_reference_action_client();

        // @brief Create the service servers for WaypointAddition.
        void set_waypoint_service_server();

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

        // @brief Handle incoming waypoint addition service requests
        //        Only accepted if waypoint action is running.
        // @param request Incoming service request containing waypoint information.
        // @param response Service response that should be populated and sent back to the caller.
        void handle_waypoint_addition_service_request(
            const std::shared_ptr<vortex_msgs::srv::WaypointAddition::Request> request,
            std::shared_ptr<vortex_msgs::srv::WaypointAddition::Response> response);

        void on_reference_filter_succeeded();

        void execution_step();

        void stop_execution_timer();

        inline geometry_msgs::msg::Pose reference_to_pose(
            const vortex_msgs::action::ReferenceFilterWaypoint::Feedback & feedback) const {
            geometry_msgs::msg::Pose pose;
            pose.position.x = feedback.reference.x;
            pose.position.y = feedback.reference.y;
            pose.position.z = feedback.reference.z;
            
            tf2::Quaternion q;
            q.setRPY(feedback.reference.roll,
                     feedback.reference.pitch,
                     feedback.reference.yaw);
            pose.orientation = tf2::toMsg(q);

            return pose;
        }


        rclcpp_action::Client<vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr reference_filter_client_;
        rclcpp_action::Server<vortex_msgs::action::WaypointManager>::SharedPtr waypoint_action_server_;
        rclcpp::Service<vortex_msgs::srv::WaypointAddition>::SharedPtr waypoint_service_server_;


        std::vector<vortex_msgs::msg::Waypoint> waypoints_{};
        std::size_t current_index_{0};
        double convergence_threshold_{1.0};

        bool persistent_action_mode_{false};
        bool non_interruptible_mode_{false};

        ReferenceFilterAction::Feedback latest_ref_feedback_;
        bool have_reference_pose_{false};
        std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;

        rclcpp::TimerBase::SharedPtr execution_timer_;
        std::shared_ptr<WaypointManagerGoalHandle> active_action_goal_;

};

} // namespace vortex::mission

#endif // WAYPOINT_MANAGER_ROS_HPP
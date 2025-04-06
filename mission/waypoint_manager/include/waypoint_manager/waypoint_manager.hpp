#ifndef WAYPOINT_MANAGER_HPP
#define WAYPOINT_MANAGER_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

class WaypointManagerNode : public rclcpp::Node {
   public:
    explicit WaypointManagerNode();

   private:
    using WaypointManagerAction = vortex_msgs::action::WaypointManager;
    using GoalHandleWaypointManager =
        rclcpp_action::ServerGoalHandle<WaypointManagerAction>;

    using ReferenceFilterAction = vortex_msgs::action::ReferenceFilterWaypoint;

    rclcpp_action::Server<WaypointManagerAction>::SharedPtr action_server_;

    rclcpp_action::Client<ReferenceFilterAction>::SharedPtr
        reference_filter_client_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    geometry_msgs::msg::Pose current_pose_;

    std::mutex mutex_;

    rclcpp::CallbackGroup::SharedPtr server_cb_group_;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;

    std::shared_ptr<GoalHandleWaypointManager> current_goal_handle_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_waypoint_index_{0};
    double switching_threshold_{0.5};
    std::string target_server_{"reference_filter"};
    bool navigation_active_{false};
    std::shared_ptr<std::promise<bool>> current_waypoint_promise_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    void setup_action_server();
    void setup_action_clients();
    void setup_subscribers();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const WaypointManagerAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypointManager> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleWaypointManager> goal_handle);

    void execute_waypoint_navigation(
        const std::shared_ptr<GoalHandleWaypointManager> goal_handle);

    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    double calculate_distance(const geometry_msgs::msg::Pose& pose1,
                              const geometry_msgs::msg::Pose& pose2);

    void send_waypoint_to_target_server(
        const geometry_msgs::msg::PoseStamped& waypoint);
};

#endif

#ifndef WAYPOINT_MANAGER_HPP
#define WAYPOINT_MANAGER_HPP

#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <condition_variable>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

class WaypointManagerNode : public rclcpp::Node {
   public:
    explicit WaypointManagerNode();
    ~WaypointManagerNode();

   private:
    using WaypointManagerAction = vortex_msgs::action::WaypointManager;
    using GoalHandleWaypointManager =
        rclcpp_action::ServerGoalHandle<WaypointManagerAction>;
    using ReferenceFilterAction = vortex_msgs::action::ReferenceFilterWaypoint;

    rclcpp_action::Server<WaypointManagerAction>::SharedPtr action_server_;
    rclcpp_action::Client<ReferenceFilterAction>::SharedPtr
        reference_filter_client_;

    std::mutex queue_mutex_;
    std::condition_variable waypoint_cv_;

    bool running_{false};
    std::thread worker_thread_;

    rclcpp::CallbackGroup::SharedPtr server_cb_group_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;

    std::shared_ptr<GoalHandleWaypointManager> current_goal_handle_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_waypoint_index_{0};
    size_t completed_waypoints_{0};
    double switching_threshold_{0.5};
    std::string target_server_{"reference_filter"};
    bool navigation_active_{false};
    rclcpp_action::GoalUUID preempted_goal_id_;

    void setup_action_server();
    void setup_action_clients();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const WaypointManagerAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypointManager> /*goal_handle*/);

    void handle_accepted(
        const std::shared_ptr<GoalHandleWaypointManager> goal_handle);

    void process_waypoints_thread();

    bool send_waypoint_to_target_server(
        const geometry_msgs::msg::PoseStamped& waypoint);
};

#endif

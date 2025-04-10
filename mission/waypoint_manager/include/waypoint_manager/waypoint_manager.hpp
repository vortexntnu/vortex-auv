#ifndef WAYPOINT_MANAGER_HPP
#define WAYPOINT_MANAGER_HPP

#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>

class WaypointManagerNode : public rclcpp::Node {
   public:
    explicit WaypointManagerNode();
    ~WaypointManagerNode() = default;

   private:
    using WaypointManagerAction = vortex_msgs::action::WaypointManager;
    using GoalHandleWaypointManager =
        rclcpp_action::ServerGoalHandle<WaypointManagerAction>;
    using ReferenceFilterAction = vortex_msgs::action::ReferenceFilterWaypoint;
    using GoalHandleReferenceFilter =
        rclcpp_action::ClientGoalHandle<ReferenceFilterAction>;

    // State enumeration
    enum class NavState { IDLE, NAVIGATING, WAITING_FOR_REFERENCE_FILTER };

    // Action server and client
    rclcpp_action::Server<WaypointManagerAction>::SharedPtr action_server_;
    rclcpp_action::Client<ReferenceFilterAction>::SharedPtr
        reference_filter_client_;

    // Subscription
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    // Current state
    geometry_msgs::msg::Pose current_pose_;
    NavState nav_state_ = NavState::IDLE;
    std::shared_ptr<GoalHandleWaypointManager> current_goal_handle_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_waypoint_index_ = 0;
    size_t completed_waypoints_ = 0;
    std::string target_server_{"reference_filter"};
    std::shared_ptr<GoalHandleReferenceFilter> reference_filter_goal_handle_;

    // Setup methods
    void setup_action_server();
    void setup_action_client();
    void setup_subscriber();

    // Action server handlers
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const WaypointManagerAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypointManager> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleWaypointManager> goal_handle);

    // Navigation methods
    void start_navigation();
    void send_next_waypoint();
    void complete_navigation(bool success);
    void publish_feedback();

    // Callbacks
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // Reference filter action callbacks
    void reference_filter_goal_response_callback(
        std::shared_future<typename GoalHandleReferenceFilter::SharedPtr>
            future);

    void reference_filter_feedback_callback(
        typename GoalHandleReferenceFilter::SharedPtr goal_handle,
        const std::shared_ptr<const ReferenceFilterAction::Feedback> feedback);

    void reference_filter_result_callback(
        const typename GoalHandleReferenceFilter::WrappedResult& result);

    // Utility methods
    double calculate_distance(const geometry_msgs::msg::Pose& pose1,
                              const geometry_msgs::msg::Pose& pose2);
};

#endif  // WAYPOINT_MANAGER_HPP

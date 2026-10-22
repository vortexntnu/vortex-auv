#ifndef REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_
#define REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/action/guidance_waypoint.hpp>
#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/reference_filter_quat.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_debug.hpp>
#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"

namespace vortex::guidance {

enum class DebugPublishMode { none, timer, on_new_goal };

class ReferenceFilterNode : public rclcpp::Node {
   public:
    explicit ReferenceFilterNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ReferenceFilterNode();

   private:
    /**
     * @brief Set the subscribers and publishers.
     */
    void set_subscribers_and_publisher();

    /**
     * @brief Set the action server.
     */
    void set_action_server();

    /**
     * @brief Initializes the reference filter with ROS parameters.
     */
    void set_refererence_filter();

    /**
     * @brief Subscribe to the mission wipe topic that triggers a system reset.
     */
    void setup_reset_subscription();

    /**
     * @brief Set up the debug goal publisher according to the
     * debug.goal_publish_mode parameter.
     */
    void setup_debug_publisher();

    /**
     * @brief Publish the current waypoint goal on the debug topic.
     */
    void publish_debug_goal();

    /**
     * @brief Publish the active waypoint and its tolerances for debugging.
     * @param wp The waypoint being executed.
     * @param tolerance The convergence tolerance of the waypoint.
     */
    void publish_waypoint_goal(
        const vortex::utils::types::Waypoint& wp,
        const vortex::utils::waypoints::ConvergenceTolerance& tolerance);

    /**
     * @brief Preempt and join the running goal so the next goal cold starts.
     * @param msg The reset message (unused).
     */
    void on_system_reset(std_msgs::msg::Empty::ConstSharedPtr msg);

    /**
     * @brief Accept all incoming goals unconditionally.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
            goal);

    /**
     * @brief Accept all cancel requests.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>> goal_handle);

    /**
     * @brief Join the old execution thread and spawn a new one for the goal.
     */
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>> goal_handle);

    /**
     * @brief Execute the action goal in a loop until convergence or
     * preemption.
     * @param goal_handle The goal handle.
     * @param retarget If true, keep the filter state from the preempted goal
     * and only change the target, else start from the measured state.
     */
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::GuidanceWaypoint>> goal_handle,
                 bool retarget);

    rclcpp_action::Server<vortex_msgs::action::GuidanceWaypoint>::SharedPtr
        action_server_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;

    ReferenceFilterParams filter_params_;

    std::unique_ptr<WaypointFollower> follower_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilterQuat>::SharedPtr
        reference_pub_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        rpy_debug_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        debug_goal_pub_;

    rclcpp::Publisher<vortex_msgs::msg::WaypointDebug>::SharedPtr
        waypoint_goal_pub_;

    bool publish_rpy_debug_{false};
    DebugPublishMode debug_mode_{DebugPublishMode::none};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    rclcpp::Subscription<vortex_msgs::msg::DVLAltitude>::SharedPtr
        altitude_sub_;

    std::chrono::milliseconds time_step_{};

    vortex::utils::types::Pose current_pose_;

    vortex::utils::types::Twist current_twist_;

    bool altitude_control_enabled_{false};
    vortex::utils::waypoints::ConvergenceTolerance default_tolerance_{0.1, 0.1};
    double current_altitude_{0.0};
    bool altitude_valid_{false};
    double altitude_lp_alpha_{0.9};

    std::mutex sensor_mutex_;

    std::atomic<bool> preempted_{false};
    std::atomic<bool> executing_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_

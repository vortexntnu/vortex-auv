/**
 * @file los_guidance_ros.hpp
 * @brief The LosGuidanceNode class initializes ROS interfaces, loads
 * configuration parameters, and runs the LOS guidance node.
 */
#ifndef LOS_GUIDANCE__ROS__LOS_GUIDANCE_ROS_HPP_
#define LOS_GUIDANCE__ROS__LOS_GUIDANCE_ROS_HPP_

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>

#include <vortex/utils/math.hpp>
#include <vortex_msgs/action/guidance_waypoint.hpp>
#include <vortex_msgs/msg/los_guidance.hpp>
#include <vortex_msgs/msg/pose_euler_stamped.hpp>
#include <vortex_msgs/msg/waypoints.hpp>
#include <vortex_msgs/srv/set_los_mode.hpp>

#include "los_guidance/lib/guidance_manager.hpp"

namespace vortex::guidance::los {

/**
 * @brief The LosGuidanceNode class initializes ROS interfaces, loads LOS
 * guidance parameters, and manages path-following execution.
 */
class LosGuidanceNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructs a LosGuidanceNode object.
     * @param options ROS node options used when creating the node.
     */
    explicit LosGuidanceNode(const rclcpp::NodeOptions& options);

   private:
    /**
     * @brief Type alias for the goal handle used by the LOS guidance action
     * server.
     */
    using GoalHandleGuidanceWaypoint =
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::GuidanceWaypoint>;

    /**
     * @brief Sets up the ROS subscribers and publishers used by the node.
     */
    void set_subscribers_and_publisher();

    /**
     * @brief Sets up the LOS guidance action server.
     */
    void set_action_server();

    /**
     * @brief Sets up the service server used for changing LOS guidance mode.
     */
    void set_service_server();

    /**
     * @brief Callback for receiving waypoint updates.
     * @param msg Received waypoint message.
     */
    void waypoint_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /**
     * @brief Callback for receiving pose updates.
     * @param msg Received pose message.
     */
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Callback for receiving odometry updates.
     * @param msg Received odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for receiving odometry updates from
     * utils/message_publisher.
     * @param msg Received odometry message.
     */
    void odom_msg_callback(
        const vortex_msgs::msg::PoseEulerStamped::SharedPtr msg);

    /**
     * @brief Handles an incoming LOS guidance action goal request.
     * @param uuid Unique identifier for the received goal.
     * @param goal Requested LOS guidance goal.
     * @return rclcpp_action::GoalResponse Response indicating whether the goal
     * is accepted.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
            goal);

    /**
     * @brief Handles cancellation of an active LOS guidance goal.
     * @param goal_handle Handle to the goal being cancelled.
     * @return rclcpp_action::CancelResponse Response indicating whether
     * cancellation is accepted.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle);

    /**
     * @brief Handles an accepted LOS guidance goal.
     * @param goal_handle Handle to the accepted goal.
     */
    void handle_accepted(
        const std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle);

    /**
     * @brief Executes the LOS guidance action.
     * @param goal_handle Handle to the active LOS guidance goal.
     */
    void execute(const std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle);

    /**
     * @brief Service callback for changing the active LOS guidance method.
     * @param request Service request containing the desired LOS mode.
     * @param response Service response indicating whether the mode change
     * succeeded.
     */
    void set_los_mode(
        const std::shared_ptr<vortex_msgs::srv::SetLosMode::Request> request,
        std::shared_ptr<vortex_msgs::srv::SetLosMode::Response> response);

    /**
     * @brief Fills a LOS guidance reference message from computed outputs.
     * @param outputs Calculated LOS guidance outputs.
     * @return vortex_msgs::msg::LOSGuidance Populated LOS guidance reference
     * message.
     */
    vortex_msgs::msg::LOSGuidance fill_los_reference(types::Outputs outputs);

    // ROS interfaces
    rclcpp_action::Server<vortex_msgs::action::GuidanceWaypoint>::SharedPtr
        action_server_;
    rclcpp::Service<vortex_msgs::srv::SetLosMode>::SharedPtr los_mode_service_;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr reference_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr
        state_debug_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        waypoint_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // State manager
    std::unique_ptr<LosGuidanceStateManager> state_manager_;

    // Node-specific state
    std::chrono::milliseconds time_step_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;
    std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle_;
    nav_msgs::msg::Odometry::SharedPtr debug_current_odom_{};
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__ROS__LOS_GUIDANCE_ROS_HPP_

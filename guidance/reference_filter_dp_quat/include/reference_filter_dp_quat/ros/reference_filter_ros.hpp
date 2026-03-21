#ifndef REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_
#define REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"

namespace vortex::guidance {

class ReferenceFilterNode : public rclcpp::Node {
   public:
    explicit ReferenceFilterNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ReferenceFilterNode();

   private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher();

    // @brief Set the action server
    void set_action_server();

    // @brief Initializes the reference filter with ROS parameters.
    void set_refererence_filter();

    /// @brief Accept all incoming goals unconditionally.
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<
            const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal);

    /// @brief Accept all cancel requests.
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    /// @brief Join the old execution thread and spawn a new one for the goal.
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    /**
     * @brief Execute the action goal in a loop until convergence or
     * preemption.
     * @param goal_handle The goal handle.
     */
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    rclcpp_action::Server<
        vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr action_server_;

    ReferenceFilterParams filter_params_;

    std::unique_ptr<WaypointFollower> follower_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    rclcpp::TimerBase::SharedPtr reference_pub_timer_;

    std::chrono::milliseconds time_step_{};

    vortex::utils::types::PoseEuler current_pose_;

    vortex::utils::types::Twist current_twist_;

    std::mutex sensor_mutex_;

    std::atomic<bool> preempted_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_

#ifndef REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_HPP_
#define REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_HPP_

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/action/guidance_waypoint.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include "reference_filter_dp/lib/waypoint_follower.hpp"

namespace vortex::guidance {

class ReferenceFilterNode : public rclcpp::Node {
   public:
    explicit ReferenceFilterNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ReferenceFilterNode();

   private:
    /** @brief Set the subscribers and publishers. */
    void set_subscribers_and_publisher();

    /** @brief Set the action server. */
    void set_action_server();

    /** @brief Initializes the reference filter with ROS parameters. */
    void set_refererence_filter();

    /** @brief Accept all incoming goals unconditionally. */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
            goal);

    /** @brief Accept all cancel requests. */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>> goal_handle);

    /**
     * @brief Swap the active goal handle and cold-start or retarget the
     * filter without tearing down the stepping thread.
     */
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>> goal_handle);

    rclcpp_action::Server<vortex_msgs::action::GuidanceWaypoint>::SharedPtr
        action_server_;

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

    /** Persistent stepping thread and lifecycle flags. */
    std::thread stepping_thread_;
    std::atomic<bool> shutdown_{false};
    std::atomic<bool> filter_initialized_{false};

    /** Currently-active goal handle; null between goals. */
    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::GuidanceWaypoint>>
        active_goal_handle_;
    std::mutex active_goal_mutex_;

    void stepping_loop();
    void start_stepping_thread();
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_HPP_

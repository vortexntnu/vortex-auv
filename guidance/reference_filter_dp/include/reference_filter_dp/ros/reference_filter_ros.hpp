#ifndef REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_HPP_
#define REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_HPP_

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include "reference_filter_dp/lib/waypoint_follower.hpp"

namespace vortex::guidance {

class ReferenceFilterNode : public rclcpp::Node {
   public:
    explicit ReferenceFilterNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher();

    // @brief Set the action server
    void set_action_server();

    // @brief Initializes the reference filter with ROS parameters.
    void set_refererence_filter();

    // @brief Callback for the reference topic
    // @param msg The reference message
    void reference_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // @brief Callback for the pose topic
    // @param msg The pose message
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // @brief Callback for the twist topic
    // @param msg The twist message
    void twist_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // @brief Handle the goal request
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<
            const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal);

    // @brief Handle the cancel request
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    // @brief Handle the accepted request
    // @param goal_handle The goal handle
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    // @brief Execute the goal
    // @param goal_handle The goal handle
    // @param generation The generation counter for this goal
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle,
                 uint64_t generation);

    Eigen::Vector18d fill_reference_state();

    Eigen::Vector6d fill_reference_goal(const geometry_msgs::msg::Pose& goal);

    Eigen::Vector6d measured_pose_vector6();

    vortex_msgs::msg::ReferenceFilter fill_reference_msg();

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

    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;

    geometry_msgs::msg::TwistWithCovarianceStamped current_twist_;

    std::atomic<uint8_t> active_mode_{vortex_msgs::msg::Waypoint::FULL_POSE};

    std::mutex mutex_;

    std::atomic<uint64_t> goal_generation_{0};

    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_HPP_

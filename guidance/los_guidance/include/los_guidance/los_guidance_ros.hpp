#ifndef REFERENCE_FILTER_ROS_HPP
#define REFERENCE_FILTER_ROS_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <los_guidance/los_guidance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/action/vtf_guidance.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/waypoints.hpp>
#include "los_guidance.hpp"

class LOSGuidanceNode : public rclcpp::Node {
   public:
    explicit LOSGuidanceNode();

   private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher();

    // @brief Set the action server
    void set_action_server();

    // @brief Callback for the reference topic
    void set_adaptive_los_guidance();

    // @brief Callback for the reference topic
    // @param msg The reference message
    void reference_callback(const vortex_msgs::msg::Waypoints::SharedPtr msg);

    // @brief Callback for the pose topic
    // @param msg The pose message
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // @brief Handle the goal request
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::VtfGuidance::Goal> goal);

    // @brief Handle the cancel request
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::VtfGuidance>>
            goal_handle);

    // @brief Handle the accepted request
    // @param goal_handle The goal handle
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                             vortex_msgs::action::VtfGuidance>> goal_handle);

    // @brief Execute the goal
    // @param goal_handle The goal handle
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::VtfGuidance>> goal_handle);

    // @brief Fill the lost waypoints
    // @param goal The goal message
    void fill_los_waypoints(const vortex_msgs::msg::Waypoints& los_waypoints);

    vortex_msgs::msg::ReferenceFilter fill_los_reference();

    rclcpp_action::Server<vortex_msgs::action::VtfGuidance>::SharedPtr
        action_server_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<vortex_msgs::msg::Waypoints>::SharedPtr reference_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::TimerBase::SharedPtr reference_pub_timer_;

    std::chrono::milliseconds time_step_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::VtfGuidance>>
        goal_handle_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    LOS::Point eta_;

    LOS::Point last_point_;

    LOS::Point next_point_;

    std::unique_ptr<AdaptiveLOSGuidance> adaptive_los_guidance_;

    double yaw_d_;

    double pitch_d_;
};

#endif

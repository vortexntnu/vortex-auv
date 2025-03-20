#ifndef REFERENCE_FILTER_ROS_HPP
#define REFERENCE_FILTER_ROS_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <reference_filter_dp/reference_filter.hpp>
#include <reference_filter_dp/reference_filter_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ReferenceFilterNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit ReferenceFilterNode();

    // @brief function for configuring the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for activating the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for cleaning up after the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for deactivating the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for shutting down the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state);

   private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher();

    // @brief Destroy the subscribers and publishers
    void destroy_topics_and_publishers();

    // @brief Set the action server
    void set_action_server();

    // @brief Destroy the action server
    void destroy_action_server();

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

    // @brief Cancel all current goals
    void cancel_all_current_goals();

    // @brief Execute the goal
    // @param goal_handle The goal handle
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    Vector18d fill_reference_state();

    Vector6d fill_reference_goal(const geometry_msgs::msg::PoseStamped& goal);

    vortex_msgs::msg::ReferenceFilter fill_reference_msg();

    rclcpp_action::Server<
        vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr action_server_;

    rclcpp_action::Client<
        vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr cancel_client_;

    ReferenceFilter reference_filter_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    rclcpp::TimerBase::SharedPtr reference_pub_timer_;

    std::chrono::milliseconds time_step_;

    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;

    geometry_msgs::msg::TwistWithCovarianceStamped current_twist_;

    // x is [eta, eta_dot, eta_dot_dot] (ref. page 337 in Fossen, 2021
    // nu and eta are 6 degrees of freedom (position and orientation in 3D
    // space)
    Vector18d x_;

    // The reference signal vector with 6 degrees of freedom [eta]
    Vector6d r_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    std::string action_server_name;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>>
        goal_handle_;

    std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>>>
        goal_handle_vector_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

#endif

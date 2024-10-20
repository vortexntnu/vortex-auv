#ifndef REFERENCE_FILTER_ROS_HPP
#define REFERENCE_FILTER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <reference_filter_dp/reference_filter.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <reference_filter_dp/reference_filter_utils.hpp>

class ReferenceFilterNode : public rclcpp::Node
{
    public:
        explicit ReferenceFilterNode();

    private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

        void reference_publisher_callback();

        void reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void set_refererence_filter();

        rclcpp_action::Server<vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr action_server_;

        ReferenceFilter reference_filter_;

        rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr reference_pub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;

        rclcpp::TimerBase::SharedPtr reference_pub_timer_;

        std::chrono::milliseconds time_step_;

        Vector18d x_;

        Vector6d r_;
};

#endif


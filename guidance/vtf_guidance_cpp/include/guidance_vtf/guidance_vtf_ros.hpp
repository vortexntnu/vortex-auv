#ifndef vtf_guidance_ros_cpp_HPP
#define vtf_guidance_ros_cpp_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/action/vtf_guidance.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/waypoints.hpp>
#include "guidance_vtf/typedefs.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GuidanceVTFNode : public rclcpp::Node {
   public:
    explicit GuidanceVTFNode();

   private:
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

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp_action::Server<vortex_msgs::action::VtfGuidance>::SharedPtr
        action_server_;

    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::VtfGuidance>>
        goal_handle_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    void guidance_publisher();

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr orca_odom_topic_;

    vortex_msgs::msg::ReferenceFilter fill_reference_msg();

    rclcpp::TimerBase::SharedPtr tau_pub_timer_;

    std::chrono::milliseconds time_step_;

    Path path_;

    State_object eta_;

    State_object eta_drone_;

    double path_index_;

    double A_param_;

    Matrix6d A_;

    Matrix6x3d B_;

    double dt_;
};
;

#endif

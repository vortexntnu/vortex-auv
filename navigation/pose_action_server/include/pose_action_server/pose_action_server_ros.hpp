#ifndef POSE_ACTION_SERVER_ROS_HPP
#define POSE_ACTION_SERVER_ROS_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/filtered_pose.hpp>

class PoseActionServerNode : public rclcpp::Node {
    using GoalHandleFilteredPose =
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::FilteredPose>;

   public:
    PoseActionServerNode();

    ~PoseActionServerNode() {};

   private:
    rclcpp_action::Server<vortex_msgs::action::FilteredPose>::SharedPtr
        action_server_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    bool is_executing_action_ = false;

    uint8_t num_measurements_;

    std::vector<geometry_msgs::msg::PoseStamped> pose_queue_;

    std::shared_ptr<GoalHandleFilteredPose> active_goal_handle_;

    void pose_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::FilteredPose::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFilteredPose> goal_handle);

    void handleAccepted(
        const std::shared_ptr<GoalHandleFilteredPose> goal_handle);
};

#endif  // POSE_ACTION_SERVER_ROS_HPP

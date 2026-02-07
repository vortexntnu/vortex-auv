#ifndef LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_
#define LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <vortex_msgs/action/landmark_convergence.hpp>
#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

#include <pose_filtering/lib/pose_track_manager.hpp>

namespace vortex::mission {

using LandmarkPollingGoalHandle =
    rclcpp_action::ServerGoalHandle<vortex_msgs::action::LandmarkPolling>;
using LandmarkConvergenceGoalHandle =
    rclcpp_action::ServerGoalHandle<vortex_msgs::action::LandmarkConvergence>;
using ReferenceFilterGoalHandle =
    rclcpp_action::ClientGoalHandle<vortex_msgs::msg::ReferenceFilter>;

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LandmarkServerNode() = default;

   private:
    void setup_ros_communicators();

    void landmark_callback(const vortex_msgs::msg::LandmarkArray& msg);

    void create_polling_action_server();

    // @brief Handle incoming landmark polling action goal requests
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_landmark_polling_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::LandmarkPolling::Goal>
            goal_msg);

    // @brief Handle requests to cancel the landmark polling action
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_landmark_polling_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::LandmarkPolling>> goal_handle);

    // @brief Handle the accepted landmark polling goal request
    // @param goal_handle The goal handle
    void handle_landmark_polling_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::LandmarkPolling>> goal_handle);

    void create_convergence_action_server();

    // @brief Handle incoming landmark convergence action goal requests
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_landmark_convergence_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::LandmarkConvergence::Goal>
            goal_msg);

    // @brief Handle requests to cancel the landmark convergence action
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_landmark_convergence_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::LandmarkConvergence>> goal_handle);

    // @brief Handle the accepted landmark convergence goal request
    // @param goal_handle The goal handle
    void handle_landmark_convergence_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::LandmarkConvergence>> goal_handle);

    void create_reference_action_client();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_pose_pub_;

    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        landmark_array_sub_;

    rclcpp_action::Server<vortex_msgs::action::LandmarkPolling>::SharedPtr
        landmark_polling_server_;

    rclcpp_action::Server<vortex_msgs::action::LandmarkConvergence>::SharedPtr
        landmark_convergence_server_;

    rclcpp_action::Client<vortex_msgs::action::ReferenceFilterWaypoint>::
        SharedPtr reference_filter_client_;

    std::unique_ptr<vortex::filtering::PoseTrackManager> track_manager_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

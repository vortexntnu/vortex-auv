#ifndef LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_
#define LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

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
using ReferenceFilterGoalHandle = rclcpp_action::ClientGoalHandle<
    vortex_msgs::action::ReferenceFilterWaypoint>;

using vortex::filtering::Landmark;

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LandmarkServerNode() = default;

   private:
    void setup_ros_communicators();

    void create_reference_publisher();

    void create_pose_subscription();

    std::vector<Landmark> ros_msg_to_landmarks(
        const vortex_msgs::msg::LandmarkArray& msg) const;

    vortex_msgs::msg::LandmarkArray tracks_to_landmark_msgs(
        uint16_t type,
        uint16_t subtype) const;

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

    // @brief Handle the accepted landmark convergence goal request
    // @param goal_handle The goal handle
    void handle_landmark_convergence_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::LandmarkConvergence>> goal_handle);

    // @brief Handle requests to cancel the landmark convergence action
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_landmark_convergence_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::LandmarkConvergence>> goal_handle);

    void create_reference_action_client();

    void create_track_manager();

    void timer_callback();

    std::shared_ptr<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>
        landmark_sub_;

    std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>
        tf_filter_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_pose_pub_;

    rclcpp_action::Server<vortex_msgs::action::LandmarkPolling>::SharedPtr
        landmark_polling_server_;

    rclcpp_action::Server<vortex_msgs::action::LandmarkConvergence>::SharedPtr
        landmark_convergence_server_;

    rclcpp_action::Client<vortex_msgs::action::ReferenceFilterWaypoint>::
        SharedPtr reference_filter_client_;

    std::unique_ptr<vortex::filtering::PoseTrackManager> track_manager_;

    std::vector<Landmark> measurements_;

    rclcpp::TimerBase::SharedPtr timer_;
    double filter_dt_seconds_{0.0};
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    bool enu_ned_rotation_{false};

    std::shared_ptr<LandmarkPollingGoalHandle> active_landmark_polling_goal_;
    std::shared_ptr<LandmarkConvergenceGoalHandle>
        active_landmark_convergence_goal_;
    std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

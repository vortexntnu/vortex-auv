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
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <vortex_msgs/action/landmark_convergence.hpp>
#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>

#include <pose_filtering/lib/pose_track_manager.hpp>
#include <pose_filtering/ros/pose_filtering_ros_conversions.hpp>

#include <mutex>
#include <optional>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <cmath>


namespace vortex::mission {

using LandmarkPollingGoalHandle =
    rclcpp_action::ServerGoalHandle<vortex_msgs::action::LandmarkPolling>;
using LandmarkConvergenceGoalHandle =
    rclcpp_action::ServerGoalHandle<vortex_msgs::action::LandmarkConvergence>;
using ReferenceFilterGoalHandle = rclcpp_action::ClientGoalHandle<
    vortex_msgs::action::ReferenceFilterWaypoint>;

using vortex::filtering::Landmark;

using RF = vortex_msgs::action::ReferenceFilterWaypoint;

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LandmarkServerNode() = default;

   private:
    void setup_ros_communicators();

    void create_reference_publisher();

    void create_pose_subscription();

    void create_odom_subscription();

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
    
    // Convergence

    void send_reference_filter_goal(
      const vortex_msgs::action::ReferenceFilterWaypoint::Goal& goal_msg,
      uint64_t seq);

    // Compute target pose = landmark_pose ⊕ convergence_offset
    geometry_msgs::msg::PoseStamped compute_target_pose(
      const vortex::filtering::Track& track,
      const geometry_msgs::msg::Pose& convergence_offset,
      const rclcpp::Time& stamp);

    // Publish reference pose (topic)
    void publish_reference_pose(const geometry_msgs::msg::PoseStamped& pose);
    // Convergence

    void create_reference_action_client();

    void create_track_manager();

    void setup_debug_publishers();

    void publish_debug_tracks();

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
    rclcpp::TimerBase::SharedPtr rf_check_timer_;
    double filter_dt_seconds_{0.0};
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    bool enu_ned_rotation_{false};

    double body_z_offset_{0.0};

    std::shared_ptr<LandmarkPollingGoalHandle> active_landmark_polling_goal_;
    std::shared_ptr<LandmarkConvergenceGoalHandle>
        active_landmark_convergence_goal_;
    std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;

    // Convergence state variables
    uint64_t convergence_session_id_{0};
    bool convergence_active_{false};
    vortex::filtering::LandmarkClassKey convergence_class_key_{};
    geometry_msgs::msg::Pose convergence_offset_{};
    double convergence_threshold_{0.0};
    double convergence_dead_reckoning_offset_{0.0};
    bool convergence_dead_reckoning_handoff_{false};
    std::optional<geometry_msgs::msg::PoseStamped> convergence_last_target_pose_;

    // Last confirmed track snapshot used to populate the action result.
    // Updated every timer tick while a convergence session is active.
    std::optional<vortex::filtering::Track> convergence_last_known_track_;

    // Track-loss timeout
    double convergence_track_loss_timeout_sec_{0.0};
    bool convergence_track_lost_{false};
    rclcpp::Time convergence_track_lost_since_{0, 0, RCL_ROS_TIME};

    void handle_convergence_update();

    // Returns the first confirmed track matching (type, subtype), or nullptr
    const vortex::filtering::Track* get_convergence_track() const;

    // Called when the convergence track is absent. Starts the track-loss
    // timeout and aborts the goal if the timeout is exceeded.
    // @return true  – caller should return immediately (track still missing)
    // @return false – should never happen (function always returns true)
    bool handle_track_loss();

    // Sends a new RF goal when none is active, or refreshes the published
    // reference pose while an RF goal is already running.
    // @param track  The current confirmed track
    void update_convergence_target(const vortex::filtering::Track& track);

    // Checks whether the vehicle is within dead-reckoning-offset of the
    // current target and, if so, sets the handoff flag.
    void check_dead_reckoning_handoff();

    // Builds a LandmarkConvergence::Result populated with the last known
    // convergence track (landmark field) and the given success flag.
    // If no track is available, landmark will be default-initialised.
    vortex_msgs::action::LandmarkConvergence::Result build_convergence_result(
        bool success) const;

    // Cache RF feedback (used to compute distance-to-target)
    std::mutex rf_fb_mtx_;
    std::optional<vortex_msgs::action::ReferenceFilterWaypoint::Feedback> last_rf_feedback_;

    // Odometry subscription — vehicle position used for dead-reckoning handoff check
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::mutex odom_mtx_;
    std::optional<geometry_msgs::msg::Point> last_odom_position_;

    bool debug_{false};
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        landmark_track_debug_pub_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

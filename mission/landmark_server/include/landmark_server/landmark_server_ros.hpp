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

#include <cmath>
#include <mutex>
#include <optional>
#include <vortex/utils/ros/ros_conversions.hpp>

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

    vortex_msgs::msg::Landmark track_to_landmark_msg(
        const vortex::filtering::Track& track) const;

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

    vortex_msgs::action::ReferenceFilterWaypoint::Goal make_rf_goal(
        const geometry_msgs::msg::Pose& target,
        double convergence_threshold) const;

    void send_reference_filter_goal(
        const vortex_msgs::action::ReferenceFilterWaypoint::Goal& goal_msg,
        uint64_t seq);

    geometry_msgs::msg::Pose compute_target_pose(
        const vortex::filtering::Track& track,
        const geometry_msgs::msg::Pose& convergence_offset);

    void create_reference_action_client();

    void create_timer();

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

    double filter_dt_seconds_{0.0};
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    bool enu_ned_rotation_{false};

    std::shared_ptr<LandmarkPollingGoalHandle> active_landmark_polling_goal_;
    std::shared_ptr<LandmarkConvergenceGoalHandle>
        active_landmark_convergence_goal_;
    std::shared_ptr<ReferenceFilterGoalHandle> active_reference_filter_goal_;

    enum class RFState { IDLE, PENDING, ACTIVE };
    RFState rf_state_{RFState::IDLE};

    // Convergence state variables
    uint64_t convergence_session_id_{0};
    bool convergence_active_{false};
    bool convergence_dead_reckoning_handoff_{false};
    uint8_t convergence_mode_{0};
    std::optional<vortex::filtering::Track> convergence_last_known_track_;

    bool convergence_track_lost_{false};
    rclcpp::Time convergence_track_lost_since_{0, 0, RCL_ROS_TIME};

    const vortex_msgs::action::LandmarkConvergence::Goal* convergence_goal()
        const {
        return active_landmark_convergence_goal_->get_goal().get();
    }

    void convergence_update();

    std::optional<vortex::filtering::Track> get_convergence_track() const;

    bool convergence_goal_active() const;

    void cancel_reference_filter_goal();

    void handle_rf_result(rclcpp_action::ResultCode code);

    bool convergence_track_timeout() const;

    void convergence_abort_track_loss();

    void convergence_handle_track_loss();

    void convergence_update_target(const vortex::filtering::Track& track);

    void convergence_check_dr_handoff();

    vortex_msgs::action::LandmarkConvergence::Result build_convergence_result(
        bool success) const;

    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::mutex odom_mtx_;
    std::optional<geometry_msgs::msg::Point> last_odom_position_;

    std::mutex measurements_mtx_;

    bool debug_{false};
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        landmark_track_debug_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrack>::SharedPtr
        convergence_landmark_debug_pub_;

    void publish_convergence_landmark_debug();
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

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

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/msg/course_frame_state.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>
#include <vortex_msgs/srv/set_course_frame.hpp>

#include <pose_filtering/lib/pose_track_manager.hpp>
#include "landmark_server/class_config.hpp"
#include "landmark_server/course_frame.hpp"
#include "landmark_server/landmark_graph.hpp"
#include "landmark_server/retained_landmarks.hpp"

#include <atomic>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vortex/utils/ros/ros_conversions.hpp>

namespace vortex::mission {

geometry_msgs::msg::PoseWithCovariance track_to_pose_with_covariance(
    const vortex::filtering::Track& track);

using LandmarkPollingGoalHandle =
    rclcpp_action::ServerGoalHandle<vortex_msgs::action::LandmarkPolling>;

using vortex::filtering::Landmark;

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LandmarkServerNode() = default;

   private:
    void setup_ros_communicators();

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

    void create_timer();

    void create_track_manager();

    void setup_reset_subscription();

    void on_system_reset(std_msgs::msg::Empty::ConstSharedPtr msg);

    void setup_debug_publishers();

    void publish_debug_tracks();

    void timer_callback();

    // --- Map: retained landmarks, course frame, publishing -----------------
    void create_map();
    void update_map();
    void publish_map();
    void publish_markers();
    void publish_course_frame();
    void reset_map();
    void handle_set_course_frame(
        const std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Request> req,
        std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Response> res);
    void handle_clear(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                      std::shared_ptr<std_srvs::srv::Empty::Response> res);
    vortex_msgs::msg::LandmarkTrack retained_to_msg(
        const RetainedLandmark& lm) const;
    vortex_msgs::msg::LandmarkTrack live_track_to_msg(
        const vortex::filtering::Track& track) const;
    vortex_msgs::msg::CourseFrameState course_frame_state_msg() const;

    // --- Smoothing backend (iSAM2, landmark_server_graph.cpp) --------------
    /// Odometry, the measurements of this tick (per map landmark) and one
    /// iSAM2 update; then the smoothed positions go into the map.
    void update_graph();
    void clear_graph();
    /// Position covariance of a measurement [m^2], odom frame: the same
    /// noise the tracker uses (class sensor noise + line-of-sight noise).
    Eigen::Matrix3d graph_measurement_cov(const Landmark& m) const;
    /// The odometry pose in target_frame_ (none if the frames differ and
    /// the TF between them is not there yet).
    std::optional<Eigen::Isometry3d> odom_in_target_frame(
        const nav_msgs::msg::Odometry& msg);

    std::shared_ptr<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>
        landmark_sub_;

    std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>
        tf_filter_;

    rclcpp_action::Server<vortex_msgs::action::LandmarkPolling>::SharedPtr
        landmark_polling_server_;

    std::unique_ptr<vortex::filtering::PoseTrackManager> track_manager_;
    vortex::filtering::TrackManagerConfig track_manager_config_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;

    std::vector<Landmark> measurements_;

    rclcpp::TimerBase::SharedPtr timer_;

    double filter_dt_seconds_{0.0};
    /// Time [s] the track filters have been predicted to (measurement time).
    std::optional<double> filter_time_sec_;
    /// Measurements discarded by intake validation (NaN, zero quaternion,
    /// invalid covariance).
    mutable std::atomic<uint64_t> dropped_measurements_{0};
    uint64_t reported_dropped_measurements_{0};
    /// Larger gaps between measurement stamps are treated as invalid.
    static constexpr double max_stamp_dt_seconds_{5.0};
    /// Smallest prediction step, for measurements older than the filter time.
    static constexpr double min_step_dt_seconds_{1e-3};
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::shared_ptr<LandmarkPollingGoalHandle> active_landmark_polling_goal_;

    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    mutable std::mutex odom_mtx_;
    std::optional<geometry_msgs::msg::Point> last_odom_position_;
    /// Latest odometry pose (stamp [s], pose in target_frame_) for the graph.
    std::optional<std::pair<double, Eigen::Isometry3d>> last_odom_pose_;
    /// target_frame_ <- odometry frame, when they differ (static).
    std::optional<Eigen::Isometry3d> target_T_odom_;

    std::mutex measurements_mtx_;

    LandmarkMapConfig map_config_;
    std::unique_ptr<RetainedLandmarks> map_;
    std::unique_ptr<CourseFrameTracker> course_;
    std::unique_ptr<LandmarkGraph> graph_;
    /// Which track each measurement of this tick went to.
    std::vector<vortex::filtering::Association> tick_associations_;
    /// Measurements of tracks that are not in the map yet, per track id.
    std::map<int, std::deque<Landmark>> pending_graph_;
    int graph_log_ticks_{0};
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        object_map_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        live_tracks_pub_;
    rclcpp::Publisher<vortex_msgs::msg::CourseFrameState>::SharedPtr
        course_frame_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<vortex_msgs::srv::SetCourseFrame>::SharedPtr
        set_course_frame_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;

    bool debug_{false};
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        landmark_track_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        landmark_pose_debug_pub_;

    uint16_t debug_landmark_type_{0};
    uint16_t debug_landmark_subtype_{0};

    void publish_debug_landmark_pose();
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

#ifndef LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_
#define LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/msg/course_frame_state.hpp>
#include <vortex_msgs/msg/course_state.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>
#include <vortex_msgs/srv/get_course.hpp>
#include <vortex_msgs/srv/set_course.hpp>
#include <vortex_msgs/srv/set_course_frame.hpp>
#include <vortex_msgs/srv/set_map_focus.hpp>

#include <pose_filtering/lib/pose_track_manager.hpp>
#include "landmark_server/class_config.hpp"
#include "landmark_server/course_frame.hpp"
#include "landmark_server/course_layout.hpp"
#include "landmark_server/landmark_graph.hpp"
#include "landmark_server/retained_landmarks.hpp"

namespace vortex::mission {

using vortex::filtering::Landmark;

/// Position and orientation covariance of a tracker track, ROS order.
geometry_msgs::msg::PoseWithCovariance track_to_pose_with_covariance(
    const vortex::filtering::Track& track);

/**
 * @brief The map: detections in, one landmark per object out.
 *
 * Every tick (timer_rate_ms): the detections since the last tick go through
 * the intake gate (course regions, focus) into the tracker, the confirmed
 * tracks into the map (course model, retained landmarks), the map through
 * the drift correction (iSAM2) and the map rules, and the map is published.
 *
 * Source files: landmark_server_ros.cpp (inputs, tick, reset),
 * _config.cpp (config files, live parameter changes), _publish.cpp (map,
 * course frame, services), _course.cpp (intake gate, course state),
 * _layout.cpp (course layouts: get_course, set_course),
 * _graph.cpp (drift correction), _polling.cpp (LandmarkPolling),
 * _markers.cpp and _debug.cpp (debug output), landmark_ros_conversion.cpp.
 */
class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    using LandmarkPolling = vortex_msgs::action::LandmarkPolling;
    using PollingGoalHandle = rclcpp_action::ServerGoalHandle<LandmarkPolling>;

    // --- Inputs (landmark_server_ros.cpp) -----------------------------------
    void create_track_manager();
    void create_detection_subscription();
    void create_odom_subscription();
    void create_reset_subscription();
    void create_timer();
    /// Detections in target_frame_ -> tracker landmarks. Invalid ones
    /// (NaN, zero quaternion, negative variance) are dropped and counted.
    std::vector<Landmark> ros_msg_to_landmarks(
        const vortex_msgs::msg::LandmarkArray& msg) const;

    // --- Tick (landmark_server_ros.cpp) -------------------------------------
    void timer_callback();
    /// One tracker update per camera frame, then hits and misses once.
    void step_tracker(std::vector<Landmark> measurements, bool had_frame);
    /// mission/wipe: everything, including the course frame.
    void on_system_reset();

    // --- Config (landmark_server_config.cpp) --------------------------------
    /// Parse the config files (the parameter overrides): map rules, course
    /// layout, graph, per-class tracker settings, debug switches.
    void load_config();
    /// Declare the config file parameters that nothing else declares, so
    /// they can be listed, dumped and changed.
    void declare_config_parameters(
        const std::map<std::string, rclcpp::ParameterValue>& overrides);
    /// Validate a parameter change. Map rules apply at the next tick, debug
    /// switches at once; tracker, graph and course settings need a restart.
    rcl_interfaces::msg::SetParametersResult on_parameters_set(
        const std::vector<rclcpp::Parameter>& params);
    /// Swap in map rules changed live (timer thread).
    void apply_pending_map_config();

    // --- Map, course frame, services (landmark_server_publish.cpp) ----------
    void create_outputs();
    void update_map();
    void publish_map();
    void publish_course_frame();
    void reset_map();
    vortex_msgs::msg::LandmarkTrack retained_to_msg(
        const RetainedLandmark& lm) const;
    vortex_msgs::msg::CourseFrameState course_frame_state_msg() const;
    void handle_set_course_frame(
        const std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Request> req,
        std::shared_ptr<vortex_msgs::srv::SetCourseFrame::Response> res);
    void handle_clear(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                      std::shared_ptr<std_srvs::srv::Empty::Response> res);

    // --- Course model (landmark_server_course.cpp) --------------------------
    /// How the course frame maps to odom now.
    CourseGeometry course_geometry() const;
    /// Before the tracker: confusable classes become their kind
    /// (reported_class keeps what the detector said), and detections of a
    /// task class far from every task that has it are dropped and counted
    /// per reason.
    void gate_measurements(std::vector<Landmark>& measurements);
    /// Tracker limits per class: the course's parts per kind, max_instances
    /// for the other classes, plus extra_tracks_per_kind.
    void apply_track_limits();
    void publish_course_state();
    void handle_set_focus(
        const std::shared_ptr<vortex_msgs::srv::SetMapFocus::Request> req,
        std::shared_ptr<vortex_msgs::srv::SetMapFocus::Response> res);

    // --- Course layouts (landmark_server_layout.cpp) ------------------------
    /// The layout directory and the layout in use, from course_file.
    void init_layouts();
    void handle_get_course(
        const std::shared_ptr<vortex_msgs::srv::GetCourse::Request> req,
        std::shared_ptr<vortex_msgs::srv::GetCourse::Response> res);
    void handle_set_course(
        const std::shared_ptr<vortex_msgs::srv::SetCourse::Request> req,
        std::shared_ptr<vortex_msgs::srv::SetCourse::Response> res);
    /// The `course` tree of another layout: its file on top of
    /// templates.yaml (only templates.yaml when it has no file yet).
    YAML::Node layout_course_tree(const std::string& layout,
                                  std::string* gui_state) const;
    /// A layout that needs a new map: the course model, the map, the
    /// tracker (its limits follow the course) and the graph start over.
    void restart_course(const CourseConfig& config);
    /// The course parameters follow the layout in use, so a parameter dump
    /// shows it.
    void sync_course_parameters();

    // --- Drift correction (landmark_server_graph.cpp) -----------------------
    /// Odometry, the measurements of this tick (under their map id) and one
    /// iSAM2 update; then the smoothed positions go into the map.
    void update_graph();
    void clear_graph();
    /// Position covariance of a measurement [m^2], odom frame: the same
    /// noise the tracker uses.
    Eigen::Matrix3d graph_measurement_cov(const Landmark& m) const;
    /// The odometry pose in target_frame_ (none while the TF between the
    /// two frames is missing).
    std::optional<Eigen::Isometry3d> odom_in_target_frame(
        const nav_msgs::msg::Odometry& msg);
    void log_graph_state() const;

    // --- LandmarkPolling action (landmark_server_polling.cpp) ---------------
    void create_polling_action_server();
    /// Answer the active goal once a confirmed track of its class exists.
    void serve_polling_goal();
    void abort_polling_goal();
    vortex_msgs::msg::LandmarkArray tracks_to_landmark_msgs(
        uint16_t type,
        uint16_t subtype) const;

    // --- Debug output (landmark_server_debug.cpp, _markers.cpp) -------------
    void create_debug_outputs();
    /// live_tracks every tick, the graph topics once a second.
    void publish_debug();
    void publish_graph_state();
    vortex_msgs::msg::LandmarkTrack live_track_to_msg(
        const vortex::filtering::Track& track) const;
    /// The map view for Foxglove/RViz (debug.markers).
    void publish_markers();

    // --- Frames and TF ------------------------------------------------------
    std::string target_frame_;
    /// Frame the graph-frame debug topics are stamped with
    /// (debug.graph_frame_id, default target_frame_).
    std::string graph_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /// target_frame_ <- odometry frame, when they differ (static).
    std::optional<Eigen::Isometry3d> target_T_odom_;

    // --- Inputs
    // ---------------------------------------------------------------
    std::shared_ptr<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>
        landmark_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>
        tf_filter_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    /// Detections since the last tick, and the detection messages received
    /// (empty ones too): a tick counts hits and misses only when one came.
    std::mutex measurements_mtx_;
    std::vector<Landmark> measurements_;
    uint64_t frames_received_{0};
    uint64_t frames_counted_{0};
    /// Detections discarded as invalid at intake.
    mutable std::atomic<uint64_t> dropped_measurements_{0};
    uint64_t reported_dropped_measurements_{0};

    mutable std::mutex odom_mtx_;
    std::optional<geometry_msgs::msg::Point> last_odom_position_;
    /// Latest odometry pose (stamp [s], pose in target_frame_).
    std::optional<std::pair<double, Eigen::Isometry3d>> last_odom_pose_;

    // --- Tracker
    // --------------------------------------------------------------
    std::unique_ptr<vortex::filtering::PoseTrackManager> track_manager_;
    vortex::filtering::TrackManagerConfig track_manager_config_;
    double filter_dt_seconds_{0.0};
    /// Time [s] the track filters have been predicted to (measurement time).
    std::optional<double> filter_time_sec_;
    /// Larger gaps between measurement stamps resynchronise the filter time.
    static constexpr double max_stamp_dt_seconds_{5.0};
    /// Smallest prediction step, for measurements older than the filter time.
    static constexpr double min_step_dt_seconds_{1e-3};
    /// Which track each measurement of this tick went to.
    std::vector<vortex::filtering::Association> tick_associations_;

    // --- Map and course
    // -------------------------------------------------------
    LandmarkMapConfig map_config_;
    std::unique_ptr<RetainedLandmarks> map_;
    std::unique_ptr<CourseFrameTracker> course_;
    /// Detections dropped at intake, by reason (since start or clear).
    std::map<std::string, int64_t> drop_counts_;
    int course_warn_ticks_{0};
    /// The `course` tree in use (templates.yaml and the layout), the layout's
    /// name and directory, and the operator GUI's state stored with it.
    YAML::Node course_tree_;
    std::string active_layout_;
    std::filesystem::path layout_dir_;
    std::string course_gui_state_;
    /// Per-class tracker settings from the config files, before the course's
    /// track limits.
    std::vector<std::pair<vortex::filtering::LandmarkClassKey,
                          vortex::filtering::LandmarkClassConfig>>
        config_class_configs_;
    /// set_course is updating the course parameters: accept them.
    bool syncing_course_parameters_{false};

    /// Live parameter changes: map rules wait for the next tick; the intake
    /// rules are read by the subscription callback.
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        parameters_cb_handle_;
    std::mutex pending_map_config_mtx_;
    std::optional<LandmarkMapConfig> pending_map_config_;
    mutable std::mutex intake_mtx_;
    IntakeConfig intake_config_;

    // --- Drift correction
    // -----------------------------------------------------
    std::unique_ptr<LandmarkGraph> graph_;
    /// Measurements of tracks that are not in the map yet, per track id.
    std::map<int, std::deque<Landmark>> pending_graph_;
    /// The correction at the previous tick; its change moves what the map
    /// keeps in odom coordinates (orientations, the course frame).
    std::optional<Eigen::Isometry3d> previous_correction_;
    double graph_update_ms_max_{0.0};
    int graph_log_ticks_{0};

    // --- Outputs
    // --------------------------------------------------------------
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        object_map_pub_;
    rclcpp::Publisher<vortex_msgs::msg::CourseFrameState>::SharedPtr
        course_frame_state_pub_;
    rclcpp::Publisher<vortex_msgs::msg::CourseState>::SharedPtr
        course_state_pub_;
    rclcpp::Service<vortex_msgs::srv::SetCourseFrame>::SharedPtr
        set_course_frame_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
    rclcpp::Service<vortex_msgs::srv::SetMapFocus>::SharedPtr set_focus_srv_;
    rclcpp::Service<vortex_msgs::srv::GetCourse>::SharedPtr get_course_srv_;
    rclcpp::Service<vortex_msgs::srv::SetCourse>::SharedPtr set_course_srv_;

    rclcpp_action::Server<LandmarkPolling>::SharedPtr polling_server_;
    std::shared_ptr<PollingGoalHandle> polling_goal_;

    // --- Debug output (debug.enable, debug.markers; switchable live)
    // ----------
    std::atomic<bool> debug_enabled_{false};
    std::atomic<bool> markers_enabled_{false};
    int debug_ticks_{0};
    rclcpp::Publisher<vortex_msgs::msg::LandmarkTrackArray>::SharedPtr
        live_tracks_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_start_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_odom_path_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr
        graph_landmarks_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        graph_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        graph_stats_pub_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

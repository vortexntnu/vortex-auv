#include <spdlog/spdlog.h>
#include <cmath>
#include "landmark_server/landmark_server_ros.hpp"

// Debug output. Off by default; config/debug.yaml (launch debug:=true)
// turns it on, and both switches can be flipped while the server runs
// (ros2 param set, or Foxglove's Parameters panel):
//   debug.enable   landmark_server/live_tracks every tick and the drift
//                  correction (landmark_server/graph/*) once a second
//   debug.markers  landmark_server/markers, the map view
// Nothing here changes the map.

namespace vortex::mission {

namespace {

geometry_msgs::msg::Pose to_pose_msg(const Eigen::Isometry3d& T) {
    geometry_msgs::msg::Pose p;
    p.position.x = T.translation().x();
    p.position.y = T.translation().y();
    p.position.z = T.translation().z();
    const Eigen::Quaterniond q(T.rotation());
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
}

double yaw_deg(const Eigen::Isometry3d& T) {
    return std::atan2(T.linear()(1, 0), T.linear()(0, 0)) * 180.0 / M_PI;
}

}  // namespace

void LandmarkServerNode::create_debug_outputs() {
    debug_enabled_ = this->declare_parameter<bool>("debug.enable", false);
    markers_enabled_ = this->declare_parameter<bool>("debug.markers", false);
    graph_frame_ = this->declare_parameter<std::string>("debug.graph_frame_id",
                                                        target_frame_);
    if (graph_frame_.empty()) {
        graph_frame_ = target_frame_;
    }

    const auto reliable = rclcpp::QoS(1).reliable();
    live_tracks_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            this->declare_parameter<std::string>("topics.live_tracks",
                                                 "landmark_server/live_tracks"),
            rclcpp::QoS(10).reliable());
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        this->declare_parameter<std::string>("topics.markers",
                                             "landmark_server/markers"),
        reliable);
    graph_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "landmark_server/graph/path", reliable);
    graph_start_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "landmark_server/graph/start_frame_path", reliable);
    graph_odom_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "landmark_server/graph/odom_path", reliable);
    graph_landmarks_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkArray>(
            "landmark_server/graph/landmarks", reliable);
    graph_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "landmark_server/graph/pose", reliable);
    graph_stats_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "landmark_server/graph/stats", rclcpp::QoS(10).reliable());

    spdlog::info("LandmarkServer: debug output {}, markers {}",
                 debug_enabled_ ? "on" : "off",
                 markers_enabled_ ? "on" : "off");
}

void LandmarkServerNode::publish_debug() {
    if (markers_enabled_) {
        publish_markers();
    }
    if (!debug_enabled_) {
        return;
    }
    vortex_msgs::msg::LandmarkTrackArray live;
    live.header.stamp = this->now();
    live.header.frame_id = target_frame_;
    for (const auto& t : track_manager_->get_tracks()) {
        live.landmark_tracks.push_back(live_track_to_msg(t));
    }
    live_tracks_pub_->publish(live);

    // The graph once a second at the default rate: the paths grow with the
    // run.
    if (graph_->config().enable && ++debug_ticks_ >= 5) {
        debug_ticks_ = 0;
        publish_graph_state();
    }
}

vortex_msgs::msg::LandmarkTrack LandmarkServerNode::live_track_to_msg(
    const vortex::filtering::Track& t) const {
    vortex_msgs::msg::LandmarkTrack msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.landmark.header = msg.header;
    msg.landmark.id = t.id;
    msg.landmark.type.value = t.class_key.type;
    msg.landmark.subtype.value = t.class_key.subtype;
    msg.landmark.pose = track_to_pose_with_covariance(t);
    msg.confirmed = t.confirmed;
    msg.hits = t.hits();
    msg.misses = t.misses();
    msg.retained = false;
    msg.has_orientation = t.has_orientation;
    msg.derived = false;
    msg.last_measurement = msg.header.stamp;
    msg.first_seen = msg.header.stamp;
    return msg;
}

void LandmarkServerNode::publish_graph_state() {
    const auto stamp = this->now();
    // Each pose carries its keyframe's stamp, so it can be compared with the
    // true pose at that time (graph_eval.py).
    const std::vector<double> stamps = graph_->keyframe_stamps();
    const auto to_path = [&](const std::vector<Eigen::Isometry3d>& poses,
                             const std::string& frame) {
        nav_msgs::msg::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = frame;
        path.poses.reserve(poses.size());
        for (std::size_t i = 0; i < poses.size(); ++i) {
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = frame;
            p.header.stamp = rclcpp::Time(
                static_cast<int64_t>(stamps[i] * 1e9), RCL_ROS_TIME);
            p.pose = to_pose_msg(poses[i]);
            path.poses.push_back(p);
        }
        return path;
    };
    // graph/path: the smoothed keyframes in the current odom frame.
    // In the graph frame (odom at the first keyframe): the smoothed
    // keyframes (start_frame_path), the raw odometry keyframes (odom_path),
    // the graph's landmarks and its latest pose, with covariances.
    graph_path_pub_->publish(
        to_path(graph_->keyframes_in_odom(), target_frame_));
    graph_start_path_pub_->publish(
        to_path(graph_->keyframes_in_graph(), graph_frame_));
    graph_odom_path_pub_->publish(to_path(graph_->keyframes_raw(), graph_frame_));

    vortex_msgs::msg::LandmarkArray landmarks;
    landmarks.header.stamp = stamp;
    landmarks.header.frame_id = graph_frame_;
    for (const auto& lm : graph_->landmarks_in_graph()) {
        vortex_msgs::msg::Landmark msg;
        msg.header = landmarks.header;
        msg.id = lm.id;
        if (const auto* retained = map_->find(lm.id)) {
            msg.type.value = retained->key.type;
            msg.subtype.value = retained->key.subtype;
        }
        msg.pose.pose.position.x = lm.position.x();
        msg.pose.pose.position.y = lm.position.y();
        msg.pose.pose.position.z = lm.position.z();
        msg.pose.pose.orientation.w = 1.0;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                msg.pose.covariance[r * 6 + c] = lm.covariance(r, c);
            }
        }
        landmarks.landmarks.push_back(std::move(msg));
    }
    graph_landmarks_pub_->publish(landmarks);

    if (const auto latest = graph_->latest_keyframe_with_covariance()) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = graph_frame_;
        pose.header.stamp =
            rclcpp::Time(static_cast<int64_t>(stamps.back() * 1e9), RCL_ROS_TIME);
        const auto& [T, P] = *latest;
        pose.pose.pose = to_pose_msg(T);
        // ROS order is (position, rotation); GTSAM's tangent is (rotation,
        // position).
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                pose.pose.covariance[r * 6 + c] = P((r + 3) % 6, (c + 3) % 6);
            }
        }
        graph_pose_pub_->publish(pose);
    }

    // [keyframes, landmarks, correction x, y [m], yaw [deg], slowest graph
    // update since the last message [ms], detector range error [%]]
    const Eigen::Isometry3d c = graph_->correction();
    std_msgs::msg::Float64MultiArray stats;
    stats.data = {static_cast<double>(graph_->keyframe_count()),
                  static_cast<double>(graph_->landmark_count()),
                  c.translation().x(),
                  c.translation().y(),
                  yaw_deg(c),
                  graph_update_ms_max_,
                  graph_->range_scale_error() * 100.0};
    graph_stats_pub_->publish(stats);
    graph_update_ms_max_ = 0.0;
}

}  // namespace vortex::mission

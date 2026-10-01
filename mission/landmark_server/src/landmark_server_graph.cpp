#include <spdlog/spdlog.h>
#include <tf2/exceptions.h>
#include <chrono>
#include <cmath>
#include <set>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

namespace {

Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Pose& p) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                    p.orientation.y, p.orientation.z)
                     .normalized()
                     .toRotationMatrix();
    return T;
}

Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Transform& t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() =
        Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
    T.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y,
                                    t.rotation.z)
                     .normalized()
                     .toRotationMatrix();
    return T;
}

}  // namespace

std::optional<Eigen::Isometry3d> LandmarkServerNode::odom_in_target_frame(
    const nav_msgs::msg::Odometry& msg) {
    const Eigen::Isometry3d odom_T_body = to_isometry(msg.pose.pose);
    if (msg.header.frame_id.empty() || msg.header.frame_id == target_frame_) {
        return odom_T_body;
    }
    // Different names for the same frame (sim: world_ned vs nautilus/odom):
    // the transform between them is static, look it up once.
    if (!target_T_odom_) {
        try {
            const auto tf = tf2_buffer_->lookupTransform(
                target_frame_, msg.header.frame_id, tf2::TimePointZero);
            target_T_odom_ = to_isometry(tf.transform);
        } catch (const tf2::TransformException& ex) {
            spdlog::warn(
                "LandmarkServer: no TF from odometry frame '{}' to '{}' yet: "
                "{}",
                msg.header.frame_id, target_frame_, ex.what());
            return std::nullopt;
        }
    }
    return *target_T_odom_ * odom_T_body;
}

Eigen::Matrix3d LandmarkServerNode::graph_measurement_cov(
    const Landmark& m) const {
    if (m.position_cov) {
        return *m.position_cov;
    }
    const vortex::filtering::LandmarkClassConfig* cfg =
        &track_manager_config_.default_class_config;
    for (const auto& [key, class_cfg] :
         track_manager_config_.per_class_configs) {
        if (key == m.class_key) {
            cfg = &class_cfg;
            break;
        }
    }
    Eigen::Matrix3d cov =
        Eigen::Matrix3d::Identity() * cfg->sens_std_dev * cfg->sens_std_dev;
    if (m.extra_position_cov) {
        cov += *m.extra_position_cov;
    } else {
        cov += Eigen::Matrix3d::Identity() * m.extra_variance;
    }
    return cov;
}

void LandmarkServerNode::clear_graph() {
    if (graph_) {
        graph_->clear();
    }
    pending_graph_.clear();
    tick_associations_.clear();
    previous_correction_.reset();
}

void LandmarkServerNode::update_graph() {
    if (!graph_ || !graph_->config().enable) {
        tick_associations_.clear();
        return;
    }
    std::optional<std::pair<double, Eigen::Isometry3d>> odom;
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        odom = last_odom_pose_;
    }
    if (odom) {
        graph_->add_odometry(odom->first, odom->second);
    }

    // Measurements wait per track until the track is in the map; then they
    // go to the graph under the map id. A track that takes over a
    // remembered landmark therefore adds to the same graph landmark: that is
    // what corrects the drift.
    for (auto& a : tick_associations_) {
        pending_graph_[a.track_id].push_back(std::move(a.measurement));
    }
    tick_associations_.clear();

    std::map<int, int> track_to_landmark;
    for (const auto& lm : map_->landmarks()) {
        if (lm.live_track_id >= 0 && !lm.derived) {
            track_to_landmark[lm.live_track_id] = lm.id;
        }
    }
    std::set<int> alive;
    for (const auto& t : track_manager_->get_tracks()) {
        alive.insert(t.id);
    }
    const auto max_pending =
        static_cast<std::size_t>(graph_->config().max_pending_per_track);
    for (auto it = pending_graph_.begin(); it != pending_graph_.end();) {
        auto& [track_id, measurements] = *it;
        const auto lm = track_to_landmark.find(track_id);
        if (lm != track_to_landmark.end()) {
            if (graph_->keyframe_count() == 0) {
                ++it;  // no odometry yet: keep them
                continue;
            }
            for (const auto& m : measurements) {
                graph_->add_measurement(lm->second, m.stamp_sec,
                                        m.pose.pos_vector(),
                                        graph_measurement_cov(m));
            }
            it = pending_graph_.erase(it);
            continue;
        }
        if (!alive.contains(track_id)) {
            it = pending_graph_.erase(it);  // deleted before it was mapped
            continue;
        }
        while (measurements.size() > max_pending) {
            measurements.pop_front();
        }
        ++it;
    }

    const auto t0 = std::chrono::steady_clock::now();
    graph_->optimize();
    graph_update_ms_max_ = std::max(graph_update_ms_max_,
                                    std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - t0)
                                        .count());

    // The correction changed: what the map keeps in odom coordinates and does
    // not get from the graph again (orientations, a locked yaw, landmarks the
    // graph does not place yet, the course frame) moves with it, so the gate
    // box turns with its posts.
    const Eigen::Isometry3d correction = graph_->correction();
    if (previous_correction_) {
        const Eigen::Isometry3d delta =
            correction * previous_correction_->inverse();
        if (!delta.isApprox(Eigen::Isometry3d::Identity(), 1e-12)) {
            map_->apply_correction(delta, [this](int id) {
                return graph_->landmark_in_odom(id).has_value();
            });
            course_->apply_correction(delta);
        }
    }
    previous_correction_ = correction;

    // The smoothed positions, in the current odom frame, replace the
    // tracker's (the orientation stays the tracker's and the rules').
    const ZLockConfig& z_lock = map_config_.map_rules.z_lock;
    for (auto& lm : map_->landmarks()) {
        if (lm.derived) {
            continue;
        }
        const auto p = graph_->landmark_in_odom(lm.id);
        if (!p) {
            continue;
        }
        lm.position = *p;
        if (z_lock.is_floor(lm.key)) {
            lm.position.z() = z_lock.floor_z;
        } else if (z_lock.is_surface(lm.key)) {
            lm.position.z() = z_lock.surface_z;
        }
    }

    // Once a second (at the default rate): trajectories and stats for
    // Foxglove.
    if (++graph_publish_ticks_ >= 5) {
        graph_publish_ticks_ = 0;
        publish_graph_state();
    }

    // Every 10 s at the default rate: how much the graph has corrected.
    if (++graph_log_ticks_ >= 50) {
        graph_log_ticks_ = 0;
        const Eigen::Isometry3d c = graph_->correction();
        const double yaw_deg =
            std::atan2(c.linear()(1, 0), c.linear()(0, 0)) * 180.0 / M_PI;
        spdlog::info(
            "LandmarkServer graph: {} keyframes, {} landmarks, correction "
            "({:.2f}, {:.2f}) m, {:.1f} deg",
            graph_->keyframe_count(), graph_->landmark_count(),
            c.translation().x(), c.translation().y(), yaw_deg);
    }
}

void LandmarkServerNode::publish_graph_state() {
    const auto stamp = this->now();
    // Each pose carries its keyframe's stamp, so it can be compared with the
    // true pose at that time (graph_eval.py).
    const std::vector<double> stamps = graph_->keyframe_stamps();
    const auto to_path = [&](const std::vector<Eigen::Isometry3d>& poses) {
        nav_msgs::msg::Path path;
        path.header.stamp = stamp;
        path.header.frame_id = target_frame_;
        path.poses.reserve(poses.size());
        for (std::size_t i = 0; i < poses.size(); ++i) {
            const auto& T = poses[i];
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = target_frame_;
            p.header.stamp = rclcpp::Time(
                static_cast<int64_t>(stamps[i] * 1e9), RCL_ROS_TIME);
            p.pose.position.x = T.translation().x();
            p.pose.position.y = T.translation().y();
            p.pose.position.z = T.translation().z();
            const Eigen::Quaterniond q(T.rotation());
            p.pose.orientation.w = q.w();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            path.poses.push_back(p);
        }
        return path;
    };
    graph_path_pub_->publish(to_path(graph_->keyframes_in_odom()));
    graph_start_path_pub_->publish(to_path(graph_->keyframes_in_graph()));

    // The graph's own estimates with their marginal covariances, in the
    // graph frame (odom at the first keyframe): for consistency checks.
    vortex_msgs::msg::LandmarkArray landmarks;
    landmarks.header.stamp = stamp;
    landmarks.header.frame_id = target_frame_;
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
        // Stamped with the keyframe's time, so it can be compared with the
        // true pose then.
        pose.header.frame_id = target_frame_;
        pose.header.stamp =
            rclcpp::Time(static_cast<int64_t>(stamps.back() * 1e9), RCL_ROS_TIME);
        const auto& [T, P] = *latest;
        pose.pose.pose.position.x = T.translation().x();
        pose.pose.pose.position.y = T.translation().y();
        pose.pose.pose.position.z = T.translation().z();
        const Eigen::Quaterniond q(T.rotation());
        pose.pose.pose.orientation.w = q.w();
        pose.pose.pose.orientation.x = q.x();
        pose.pose.pose.orientation.y = q.y();
        pose.pose.pose.orientation.z = q.z();
        // ROS order is (position, rotation); GTSAM's tangent is (rotation,
        // position).
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                pose.pose.covariance[r * 6 + c] = P((r + 3) % 6, (c + 3) % 6);
            }
        }
        graph_pose_pub_->publish(pose);
    }
    graph_odom_path_pub_->publish(to_path(graph_->keyframes_raw()));

    const Eigen::Isometry3d c = graph_->correction();
    std_msgs::msg::Float64MultiArray stats;
    stats.data = {static_cast<double>(graph_->keyframe_count()),
                  static_cast<double>(graph_->landmark_count()),
                  c.translation().x(),
                  c.translation().y(),
                  std::atan2(c.linear()(1, 0), c.linear()(0, 0)) * 180.0 / M_PI,
                  graph_update_ms_max_};
    graph_stats_pub_->publish(stats);
    graph_update_ms_max_ = 0.0;
}

}  // namespace vortex::mission

#include <spdlog/spdlog.h>
#include <tf2/exceptions.h>
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

    graph_->optimize();

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

}  // namespace vortex::mission

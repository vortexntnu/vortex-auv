#include "landmark_server/retained_landmarks.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vortex::mission {

double RetainedLandmark::yaw() const {
    const Eigen::Quaterniond q = orientation.normalized();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

RetainedLandmarks::RetainedLandmarks(LandmarkMapConfig config)
    : config_(std::move(config)) {}

void RetainedLandmarks::clear() {
    landmarks_.clear();
    rejected_ = 0;
}

const RetainedLandmark* RetainedLandmarks::find(int id) const {
    const auto it = std::find_if(landmarks_.begin(), landmarks_.end(),
                                 [&](const auto& l) { return l.id == id; });
    return it == landmarks_.end() ? nullptr : &*it;
}

RetainedLandmark& RetainedLandmarks::upsert_derived(
    const std::string& slot,
    const vortex::filtering::LandmarkClassKey& key,
    double now) {
    for (auto& lm : landmarks_) {
        if (lm.derived && lm.derived_slot == slot) {
            lm.key = key;
            return lm;
        }
    }
    RetainedLandmark lm;
    lm.id = next_id_++;
    lm.key = key;
    lm.derived = true;
    lm.derived_slot = slot;
    lm.first_seen = now;
    lm.last_measurement = now;
    landmarks_.push_back(std::move(lm));
    return landmarks_.back();
}

void RetainedLandmarks::update_from_track(RetainedLandmark& lm,
                                          const vortex::filtering::Track& track,
                                          double now) const {
    lm.live_track_id = track.id;
    lm.position = track.nominal_state.pos;
    if (track.has_orientation && !lm.yaw_locked) {
        lm.orientation = track.nominal_state.ori;
        lm.has_orientation = true;
    }

    // The tracker keeps a 6x6 covariance of the error state (position first).
    lm.covariance = track.error_state.cov();
    lm.hits = track.hits();
    lm.misses = track.misses();

    if (!track.hit_history.empty() && track.hit_history.back()) {
        lm.last_measurement = now;
        ++lm.observations;
    }
}

bool RetainedLandmarks::near_large_structure(const Eigen::Vector3d& position,
                                             double distance) const {
    return std::any_of(landmarks_.begin(), landmarks_.end(),
                       [&](const RetainedLandmark& l) {
                           return config_.is_large_structure(l.key) &&
                                  (l.position - position).norm() < distance;
                       });
}

void RetainedLandmarks::forget_expired(double now) {
    landmarks_.erase(
        std::remove_if(landmarks_.begin(), landmarks_.end(),
                       [&](const RetainedLandmark& l) {
                           if (l.derived || l.live_track_id >= 0) {
                               return false;
                           }
                           const ClassRule& rule = config_.rule_for(l.key);
                           if (rule.retain_forever) {
                               return false;
                           }
                           if (rule.keep_after_observations > 0 &&
                               l.observations >= rule.keep_after_observations) {
                               return false;
                           }
                           return now - l.last_measurement > rule.retain_sec;
                       }),
        landmarks_.end());
}

void RetainedLandmarks::update(
    const std::vector<vortex::filtering::Track>& confirmed,
    double now,
    const PositionFilter& position_allowed) {
    // Landmarks whose live track is gone are only remembered from now on.
    for (auto& lm : landmarks_) {
        if (lm.live_track_id < 0) {
            continue;
        }
        const bool alive = std::any_of(
            confirmed.begin(), confirmed.end(),
            [&](const auto& t) { return t.id == lm.live_track_id; });
        if (!alive) {
            lm.live_track_id = -1;
        }
    }

    // Lane bounds tighten when the gate locks: drop what is now outside.
    if (position_allowed) {
        landmarks_.erase(
            std::remove_if(landmarks_.begin(), landmarks_.end(),
                           [&](const RetainedLandmark& l) {
                               return !l.derived &&
                                      !position_allowed(l.position);
                           }),
            landmarks_.end());
    }

    for (const auto& track : confirmed) {
        // 1. A track that is already followed.
        auto followed = std::find_if(
            landmarks_.begin(), landmarks_.end(),
            [&](const auto& l) { return l.live_track_id == track.id; });
        if (followed != landmarks_.end()) {
            update_from_track(*followed, track, now);
            continue;
        }

        const Eigen::Vector3d position = track.nominal_state.pos;
        if (position_allowed && !position_allowed(position)) {
            ++rejected_;
            continue;
        }

        const ClassRule& rule = config_.rule_for(track.class_key);
        const double gate = rule.instance_gate_m > 0.0
                                ? rule.instance_gate_m
                                : config_.plausibility_radius_m;
        // Classes with a single instance accept a wider radius, so that the
        // one object is not lost to a slightly different estimate.
        const double adopt_radius =
            rule.max_instances == 1
                ? std::max(gate, config_.plausibility_radius_m)
                : gate;

        // 2. A remembered landmark of the same class takes the track over.
        RetainedLandmark* best = nullptr;
        double best_dist = std::numeric_limits<double>::infinity();
        int instances = 0;
        for (auto& lm : landmarks_) {
            if (lm.key != track.class_key || lm.derived) {
                continue;
            }
            ++instances;
            if (lm.live_track_id >= 0) {
                continue;
            }
            const double d = (lm.position - position).norm();
            if (d <= adopt_radius && d < best_dist) {
                best = &lm;
                best_dist = d;
            }
        }
        if (best != nullptr) {
            update_from_track(*best, track, now);
            continue;
        }

        // 3. A new landmark, if the class has room and the rules allow it.
        if (instances >= rule.max_instances) {
            ++rejected_;
            continue;
        }
        if (rule.min_distance_to_large_structures_m > 0.0 &&
            near_large_structure(position,
                                 rule.min_distance_to_large_structures_m)) {
            ++rejected_;
            continue;
        }

        RetainedLandmark lm;
        lm.id = next_id_++;
        lm.key = track.class_key;
        lm.first_seen = now;
        lm.last_measurement = now;
        update_from_track(lm, track, now);
        landmarks_.push_back(std::move(lm));
    }

    forget_expired(now);
}

}  // namespace vortex::mission

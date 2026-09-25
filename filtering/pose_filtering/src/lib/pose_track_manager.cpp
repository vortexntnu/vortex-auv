#include "pose_filtering/lib/pose_track_manager.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <probability/multi_var_gauss.hpp>
#include <ranges>
#include <vortex/utils/math.hpp>
#include "pose_filtering/lib/hungarian.hpp"
#include "pose_filtering/lib/typedefs.hpp"

namespace vortex::filtering {

PoseTrackManager::PoseTrackManager(const TrackManagerConfig& config)
    : track_id_counter_(0), config_(config) {
    validate_config(config_);
}

void PoseTrackManager::step(std::vector<Landmark>& measurements, double dt) {
    update(measurements, dt);
    end_cycle();
}

PoseGate6D PoseTrackManager::gate_for(const LandmarkClassConfig& cfg) const {
    PoseGate6D gate;
    gate.min_pos_error = cfg.min_pos_error;
    gate.max_pos_error = cfg.max_pos_error;
    gate.min_ori_error = cfg.min_ori_error;
    gate.max_ori_error = cfg.max_ori_error;
    gate.mahalanobis_threshold = cfg.mahalanobis_threshold;
    return gate;
}

SensorMod PoseTrackManager::sensor_model_for(
    const LandmarkClassConfig& cfg,
    const Landmark* measurement) const {
    const double sens_var = cfg.sens_std_dev * cfg.sens_std_dev;
    Eigen::Matrix<double, 6, 6> sensor_cov =
        Eigen::Matrix<double, 6, 6>::Identity() * sens_var;
    if (measurement != nullptr) {
        // Noisier measurements (for instance far away) weigh less. Only the
        // position noise; the orientation noise is the class value.
        sensor_cov.topLeftCorner<3, 3>() += extra_position_cov(*measurement);
    }
    return SensorMod(sensor_cov);
}

Eigen::Matrix3d PoseTrackManager::extra_position_cov(const Landmark& m) {
    if (m.extra_position_cov) {
        return *m.extra_position_cov;
    }
    return Eigen::Matrix3d::Identity() * m.extra_variance;
}

std::vector<int> PoseTrackManager::associate(
    const std::vector<Landmark>& measurements,
    double dt) const {
    std::vector<int> assignment(tracks_.size(), -1);

    std::vector<LandmarkClassKey> classes;
    for (const Landmark& m : measurements) {
        if (std::ranges::find(classes, m.class_key) == classes.end()) {
            classes.push_back(m.class_key);
        }
    }

    for (const LandmarkClassKey& key : classes) {
        std::vector<int> track_idx;
        for (int t = 0; t < static_cast<int>(tracks_.size()); ++t) {
            if (tracks_[t].class_key == key) {
                track_idx.push_back(t);
            }
        }
        std::vector<Eigen::Index> meas_idx;
        for (Eigen::Index i = 0;
             i < static_cast<Eigen::Index>(measurements.size()); ++i) {
            if (measurements[i].class_key == key) {
                meas_idx.push_back(i);
            }
        }
        if (track_idx.empty()) {
            continue;
        }

        const auto& cfg = cfg_for(key);
        const PoseGate6D gate = gate_for(cfg);
        const DynMod dyn_mod(cfg.dyn_std_dev);
        const int n_t = static_cast<int>(track_idx.size());
        const int n_m = static_cast<int>(meas_idx.size());
        Eigen::MatrixXd cost = Eigen::MatrixXd::Zero(n_t, n_m);
        Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> allowed =
            Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>::Constant(
                n_t, n_m, false);

        // Squared Mahalanobis distance with the innovation covariance of
        // each pair (the measurement noise depends on the measurement).
        for (int a = 0; a < n_t; ++a) {
            const Track& track = tracks_[track_idx[a]];
            for (int b = 0; b < n_m; ++b) {
                const Landmark& m = measurements[meas_idx[b]];
                const auto z = compute_measurement_residuals(
                                   track, measurements, {meas_idx[b]})
                                   .matrix()
                                   .col(0)
                                   .eval();
                const auto pred = PDAF::predict(
                    dyn_mod, sensor_model_for(cfg, &m), dt, track.error_state);
                if (gate(z, pred.z_pred)) {
                    allowed(a, b) = true;
                    const double d = pred.z_pred.mahalanobis_distance(z);
                    cost(a, b) = d * d;
                }
            }
        }

        // With a gate on d <= gamma every allowed pair is cheaper than
        // leaving both unpaired (gamma^2).
        double unpaired = cfg.mahalanobis_threshold * cfg.mahalanobis_threshold;
        if (!std::isfinite(unpaired)) {
            unpaired =
                allowed.any()
                    ? 2.0 * (allowed.cast<double>() * cost.array()).maxCoeff() +
                          1.0
                    : 1.0;
        }
        const auto pairs = associate_gnn(cost, allowed, unpaired);
        for (int a = 0; a < n_t; ++a) {
            if (pairs[a] >= 0) {
                assignment[track_idx[a]] = static_cast<int>(meas_idx[pairs[a]]);
            }
        }
    }
    return assignment;
}

void PoseTrackManager::update(std::vector<Landmark>& measurements, double dt) {
    sort_tracks_by_priority();
    associations_.clear();
    const std::vector<int> assignment = associate(measurements, dt);

    std::vector<Eigen::Index> used;
    for (int t = 0; t < static_cast<int>(tracks_.size()); ++t) {
        Track& track = tracks_[t];
        const auto& cfg = cfg_for(track);
        const DynMod dyn_mod(cfg.dyn_std_dev);

        PDAF::Config pdaf_cfg;
        pdaf_cfg.pdaf.mahalanobis_threshold = cfg.mahalanobis_threshold;
        pdaf_cfg.pdaf.prob_of_detection = cfg.prob_of_detection;
        pdaf_cfg.pdaf.clutter_intensity = cfg.clutter_intensity;

        // The track gets its own measurement only (0 or 1 column), so PDAF
        // no longer averages over measurements that belong to other tracks.
        const int mi = assignment[t];
        const Landmark* m = mi >= 0 ? &measurements[mi] : nullptr;
        std::vector<Eigen::Index> idx;
        if (m != nullptr) {
            idx.push_back(mi);
        }
        const auto z = compute_measurement_residuals(track, measurements, idx);

        auto pdaf_output =
            PDAF::step(dyn_mod, sensor_model_for(cfg, m), dt, track.error_state,
                       z, pdaf_cfg, gate_for(cfg));
        const bool hit = pdaf_output.gated_measurements.any();

        // Without orientation in the measurement (or the track), the
        // orientation residual is a placeholder zero. Keep the predicted
        // orientation so it does not look measured. Position and
        // orientation are uncorrelated here (block-diagonal models), so this
        // equals a position-only update.
        if (hit && !(m->has_orientation && track.has_orientation)) {
            const auto& pred = pdaf_output.x_pred;
            pdaf_output.x_post.mean().tail<3>() = pred.mean().tail<3>();
            pdaf_output.x_post.cov().block<3, 3>(3, 3) =
                pred.cov().block<3, 3>(3, 3);
            pdaf_output.x_post.cov().block<3, 3>(0, 3) =
                pred.cov().block<3, 3>(0, 3);
            pdaf_output.x_post.cov().block<3, 3>(3, 0) =
                pred.cov().block<3, 3>(3, 0);
        }

        adopt_orientation(track, measurements, idx,
                          pdaf_output.gated_measurements);

        track.error_state = pdaf_output.x_post;
        inject_and_reset(track);
        track.hit_in_cycle = track.hit_in_cycle || hit;
        if (hit) {
            used.push_back(mi);
            associations_.push_back({track.id, measurements[mi]});
        }
    }

    std::ranges::sort(used, std::greater<Eigen::Index>());
    for (Eigen::Index i : used) {
        measurements.erase(measurements.begin() + i);
    }
    create_tracks(measurements);
}

void PoseTrackManager::end_cycle() {
    for (Track& track : tracks_) {
        // A track created in this cycle already has its first hit.
        if (!track.created_in_cycle) {
            record_hit_miss(track, track.hit_in_cycle);
        }
        track.hit_in_cycle = false;
        track.created_in_cycle = false;
    }
    delete_tracks();
    confirm_tracks();
}

Eigen::Array<double, 6, Eigen::Dynamic>
PoseTrackManager::compute_measurement_residuals(
    const Track& track,
    const std::vector<Landmark>& measurements,
    const std::vector<Eigen::Index>& indices) const {
    PDAF::Arr_zXd Z(6, indices.size());

    for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(indices.size());
         ++k) {
        const Landmark& measurement = measurements[indices[k]];
        const Pose& meas_pose = measurement.pose;

        const Eigen::Vector3d dp =
            meas_pose.pos_vector() - track.nominal_state.pos;
        // Position-only detections, or a track that has no orientation yet,
        // carry no information about the orientation: leave it untouched.
        const bool use_orientation =
            measurement.has_orientation && track.has_orientation;
        const Eigen::Vector3d dtheta =
            use_orientation ? so3_log_quat(meas_pose.ori_quaternion() *
                                           track.nominal_state.ori.conjugate())
                            : Eigen::Vector3d::Zero().eval();

        Z.matrix().col(k).head<3>() = dp;
        Z.matrix().col(k).tail<3>() = dtheta;
    }

    return Z;
}

void PoseTrackManager::adopt_orientation(
    Track& track,
    const std::vector<Landmark>& measurements,
    const std::vector<Eigen::Index>& global_indices,
    const Eigen::Array<bool, 1, Eigen::Dynamic>& mask) const {
    if (track.has_orientation) {
        return;
    }
    for (Eigen::Index k = 0;
         k < static_cast<Eigen::Index>(global_indices.size()); ++k) {
        const Landmark& m = measurements[global_indices[k]];
        if (mask(k) && m.has_orientation) {
            track.nominal_state.ori = m.pose.ori_quaternion();
            track.has_orientation = true;
            return;
        }
    }
}

void PoseTrackManager::inject_and_reset(Track& track) {
    const Eigen::Matrix<double, 6, 1> delta = track.error_state.mean();

    track.nominal_state.pos += delta.head<3>();

    track.nominal_state.ori =
        so3_exp_quat(delta.tail<3>()) * track.nominal_state.ori;
    track.nominal_state.ori.normalize();

    // First-order reset Jacobian for left-injected SO(3) error:
    // G_theta ≈ I - 0.5 * skew(delta_theta)
    Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Identity();
    G.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() -
        0.5 * vortex::utils::math::get_skew_symmetric_matrix(delta.tail<3>());

    track.error_state.cov() = G * track.error_state.cov() * G.transpose();

    track.error_state.mean().setZero();
}

Eigen::Vector3d PoseTrackManager::so3_log_quat(
    const Eigen::Quaterniond& q_in) const {
    Eigen::Quaterniond q = q_in.normalized();

    if (q.w() < 0.0) {
        // Quat sign ambiguity q == -q
        // Enforce consistent sign convention
        q.coeffs() *= -1.0;
    }

    double norm_v = q.vec().norm();

    if (norm_v < 1e-6) {
        // Small-angle approximation of log map:
        // for θ → 0, log(q) ≈ 2 * v since q ≈ [1, v]
        return 2.0 * q.vec();
    }

    double theta = 2.0 * std::atan2(norm_v, q.w());
    return theta * q.vec() / norm_v;
}

Eigen::Quaterniond PoseTrackManager::so3_exp_quat(
    const Eigen::Vector3d& rvec) const {
    double theta = rvec.norm();

    if (theta < 1e-6) {
        return Eigen::Quaterniond(1.0, 0.5 * rvec.x(), 0.5 * rvec.y(),
                                  0.5 * rvec.z())
            .normalized();
    }

    Eigen::Vector3d axis = rvec / theta;
    double half = 0.5 * theta;

    return Eigen::Quaterniond(std::cos(half), axis.x() * std::sin(half),
                              axis.y() * std::sin(half),
                              axis.z() * std::sin(half));
}

void PoseTrackManager::create_tracks(
    const std::vector<Landmark>& measurements) {
    tracks_.reserve(tracks_.size() + measurements.size());

    auto make_track = [this](const Landmark& measurement) {
        const auto& cfg = cfg_for(measurement);

        Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Zero();
        P0.block<3, 3>(0, 0).setIdentity();
        P0.block<3, 3>(3, 3).setIdentity();

        P0.block<3, 3>(0, 0) *= cfg.init_pos_std * cfg.init_pos_std;
        // A new track is no surer than the measurement it comes from.
        P0.block<3, 3>(0, 0) +=
            sensor_model_for(cfg, &measurement).R().topLeftCorner<3, 3>();
        P0.block<3, 3>(3, 3) *= cfg.init_ori_std * cfg.init_ori_std;

        Track t{.id = track_id_counter_++,
                .class_key = measurement.class_key,
                .nominal_state =
                    NominalState{.pos = measurement.pose.pos_vector(),
                                 .ori = measurement.pose.ori_quaternion()},
                .error_state = vortex::prob::Gauss6d(
                    Eigen::Matrix<double, 6, 1>::Zero(), P0),
                .confirmed = false};
        t.has_orientation = measurement.has_orientation;
        t.hit_history.push_back(true);
        t.hit_in_cycle = true;
        t.created_in_cycle = true;
        return t;
    };

    for (const Landmark& m : measurements) {
        tracks_.push_back(make_track(m));
        associations_.push_back({tracks_.back().id, m});
    }
}

void PoseTrackManager::record_hit_miss(Track& track, bool hit) {
    const auto& nm = cfg_for(track).nm;
    int max_window = std::max(nm.confirm_m, nm.delete_m);
    track.hit_history.push_back(hit);
    while (static_cast<int>(track.hit_history.size()) > max_window) {
        track.hit_history.pop_front();
    }
}

void PoseTrackManager::confirm_tracks() {
    for (Track& track : tracks_) {
        if (track.confirmed) {
            continue;
        }
        const auto& nm = cfg_for(track).nm;
        if (static_cast<int>(track.hit_history.size()) < nm.confirm_m) {
            continue;
        }
        int recent_hits = 0;
        auto it = track.hit_history.rbegin();
        for (int i = 0; i < nm.confirm_m; ++i, ++it) {
            if (*it)
                ++recent_hits;
        }
        if (recent_hits >= nm.confirm_n) {
            track.confirmed = true;
        }
    }
}

void PoseTrackManager::delete_tracks() {
    auto new_end = std::ranges::remove_if(tracks_, [this](const Track& track) {
        const auto& nm = cfg_for(track).nm;
        if (static_cast<int>(track.hit_history.size()) < nm.delete_m) {
            return false;
        }
        int recent_misses = 0;
        auto it = track.hit_history.rbegin();
        for (int i = 0; i < nm.delete_m; ++i, ++it) {
            if (!*it)
                ++recent_misses;
        }
        return recent_misses >= nm.delete_n;
    });
    tracks_.erase(new_end.begin(), new_end.end());
}

void PoseTrackManager::sort_tracks_by_priority() {
    std::ranges::sort(tracks_, [](const Track& a, const Track& b) {
        if (a.confirmed != b.confirmed)
            return a.confirmed > b.confirmed;
        return a.hits() > b.hits();
    });
}

}  // namespace vortex::filtering

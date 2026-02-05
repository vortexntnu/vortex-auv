#include "pose_filtering/lib/pose_track_manager.hpp"
#include <algorithm>
#include <probability/multi_var_gauss.hpp>
#include <ranges>
#include <vortex/utils/math.hpp>
#include "pose_filtering/lib/typedefs.hpp"

namespace vortex::filtering {

PoseTrackManager::PoseTrackManager(const TrackManagerConfig& config)
    : track_id_counter_(0), config_(config) {}

void PoseTrackManager::step(std::vector<Landmark>& measurements, double dt) {
    sort_tracks_by_priority();
    std::cout << "Step called with " << tracks_.size() << " tracks and "
              << measurements.size() << " measurements." << std::endl;
    for (Track& track : tracks_) {
        const auto& cfg = cfg_for(track);
        std::cout << "Processing track " << track.id << " with type "
                  << track.type << " and subtype " << track.subtype
                  << std::endl;
        auto type_gate_indices = type_gate_measurements(track, measurements);
        auto Z = compute_measurement_residuals(track, measurements,
                                               type_gate_indices);

        PoseGate6D gate;
        gate.min_pos_error = cfg.min_pos_error;
        gate.max_pos_error = cfg.max_pos_error;
        gate.min_ori_error = cfg.min_ori_error;
        gate.max_ori_error = cfg.max_ori_error;
        gate.mahalanobis_threshold = cfg.mahalanobis_threshold;

        DynMod dyn_mod(cfg.dyn_std_dev);
        SensorMod sensor_mod(cfg.sens_std_dev);

        vortex::filter::IPDA<DynMod, SensorMod>::Config ipda_cfg;
        ipda_cfg.ipda.estimate_clutter = cfg.estimate_clutter;
        ipda_cfg.ipda.prob_of_survival = cfg.prob_of_survival;
        ipda_cfg.pdaf.mahalanobis_threshold = cfg.mahalanobis_threshold;
        ipda_cfg.pdaf.prob_of_detection = cfg.prob_of_detection;
        ipda_cfg.pdaf.clutter_intensity = cfg.clutter_intensity;

        const IPDA::State state_est_prev{
            .x_estimate = track.error_state,
            .existence_probability = track.existence_probability};

        auto ipda_output = IPDA::step(dyn_mod, sensor_mod, dt, state_est_prev,
                                      Z, ipda_cfg, gate);

        track.error_state = ipda_output.state.x_estimate;
        track.existence_probability = ipda_output.state.existence_probability;
        inject_and_reset(track);
        erase_gated_measurements(measurements, type_gate_indices,
                                 ipda_output.gated_measurements);
    }
    confirm_tracks();
    delete_tracks();
    create_tracks(measurements);
}

std::vector<Eigen::Index> PoseTrackManager::type_gate_measurements(
    const Track& track,
    const std::vector<Landmark>& measurements) const {
    std::vector<Eigen::Index> idx;
    idx.reserve(measurements.size());

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(measurements.size());
         ++i) {
        const auto& m = measurements[i];
        if (m.class_key.type == track.type &&
            m.class_key.subtype == track.subtype) {
            idx.push_back(i);
        }
    }
    return idx;
}

Eigen::Array<double, 6, Eigen::Dynamic>
PoseTrackManager::compute_measurement_residuals(
    const Track& track,
    const std::vector<Landmark>& measurements,
    const std::vector<Eigen::Index>& indices) const {
    IPDA::Arr_zXd Z(6, indices.size());

    for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(indices.size());
         ++k) {
        const Pose& meas_pose = measurements[indices[k]].pose;

        const Eigen::Vector3d dp =
            meas_pose.pos_vector() - track.nominal_state.pos;
        const Eigen::Vector3d dtheta = so3_log_quat(
            meas_pose.ori_quaternion() * track.nominal_state.ori.conjugate());

        Z.matrix().col(k).head<3>() = dp;
        Z.matrix().col(k).tail<3>() = dtheta;
    }

    return Z;
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

void PoseTrackManager::erase_gated_measurements(
    std::vector<Landmark>& measurements,
    const std::vector<Eigen::Index>& global_indices,
    const Eigen::Array<bool, 1, Eigen::Dynamic>& mask) const {
    std::vector<Eigen::Index> to_erase;
    to_erase.reserve(global_indices.size());

    for (Eigen::Index k = 0;
         k < static_cast<Eigen::Index>(global_indices.size()); ++k) {
        if (mask(k)) {  // <-- confirm semantics: true == "use/associated"
            to_erase.push_back(global_indices[k]);
        }
    }

    std::sort(to_erase.begin(), to_erase.end(), std::greater<Eigen::Index>());
    for (Eigen::Index idx : to_erase) {
        measurements.erase(measurements.begin() + idx);
    }
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
        P0.block<3, 3>(3, 3) *= cfg.init_ori_std * cfg.init_ori_std;

        return Track{.id = track_id_counter_++,
                     .type = measurement.class_key.type,
                     .subtype = measurement.class_key.subtype,
                     .nominal_state =
                         NominalState{.pos = measurement.pose.pos_vector(),
                                      .ori = measurement.pose.ori_quaternion()},
                     .error_state = vortex::prob::Gauss6d(
                         Eigen::Matrix<double, 6, 1>::Zero(), P0),
                     .existence_probability =
                         config_.existence.initial_existence_probability,
                     .confirmed = false};
    };

    for (const Landmark& m : measurements) {
        tracks_.push_back(make_track(m));
    }
}

void PoseTrackManager::confirm_tracks() {
    for (Track& track : tracks_) {
        if (!track.confirmed && track.existence_probability >=
                                    config_.existence.confirmation_threshold) {
            track.confirmed = true;
        }
    }
}

void PoseTrackManager::delete_tracks() {
    auto new_end = std::ranges::remove_if(tracks_, [this](Track& track) {
        return track.existence_probability <
               config_.existence.deletion_threshold;
    });
    tracks_.erase(new_end.begin(), new_end.end());
}

void PoseTrackManager::sort_tracks_by_priority() {
    std::ranges::sort(tracks_, [](const Track& a, const Track& b) {
        if (a.confirmed != b.confirmed)
            return a.confirmed > b.confirmed;
        return a.existence_probability > b.existence_probability;
    });
}

}  // namespace vortex::filtering

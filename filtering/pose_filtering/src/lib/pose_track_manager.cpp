#include "pose_filtering/lib/pose_track_manager.hpp"
#include <algorithm>
#include <ranges>
#include <vortex/utils/math.hpp>

namespace vortex::filtering {

PoseTrackManager::PoseTrackManager(const TrackManagerConfig& config)
    : track_id_counter_(0),
      dyn_mod_(config.dyn_mod.std_dev),
      sensor_mod_(config.sensor_mod.std_dev) {
    ipda_config_ = config.ipda;
    existence_config_ = config.existence;
    max_angle_gate_threshold_ = config.max_angle_gate_threshold;
}

void PoseTrackManager::step(std::vector<Pose>& measurements, double dt) {
    sort_tracks_by_priority();

    for (Track& track : tracks_) {
        auto angular_gate_indices =
            angular_gate_measurements(track, measurements);

        Eigen::Matrix<double, 3, Eigen::Dynamic> Z =
            build_position_matrix(measurements, angular_gate_indices);

        const IPDA::State state_est_prev{
            .x_estimate = track.state,
            .existence_probability = track.existence_probability};

        auto ipda_output = IPDA::step(dyn_mod_, sensor_mod_, dt, state_est_prev,
                                      Z, ipda_config_);

        track.state = ipda_output.state.x_estimate;
        track.existence_probability = ipda_output.state.existence_probability;

        std::vector<Eigen::Index> consumed_indices;
        auto used_quaternions = collect_used_quaternions(
            measurements, angular_gate_indices, ipda_output.gated_measurements,
            consumed_indices);

        update_track_orientation(track, used_quaternions);

        erase_gated_measurements(measurements, consumed_indices);
    }

    confirm_tracks();
    delete_tracks();
    create_tracks(measurements);
}

std::vector<Eigen::Index> PoseTrackManager::angular_gate_measurements(
    const Track& track,
    const std::vector<Pose>& measurements) const {
    std::vector<Eigen::Index> indices;
    indices.reserve(measurements.size());

    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(measurements.size());
         ++i) {
        if (track.angular_distance(measurements[i]) <
            max_angle_gate_threshold_) {
            indices.push_back(i);
        }
    }
    return indices;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PoseTrackManager::build_position_matrix(
    const std::vector<Pose>& measurements,
    const std::vector<Eigen::Index>& indices) const {
    Eigen::Matrix<double, 3, Eigen::Dynamic> Z(3, indices.size());
    for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(indices.size());
         ++k) {
        Z.col(k) = measurements[indices[k]].pos_vector();
    }
    return Z;
}

std::vector<Eigen::Quaterniond> PoseTrackManager::collect_used_quaternions(
    const std::vector<Pose>& measurements,
    const std::vector<Eigen::Index>& angular_gate_indices,
    const Eigen::Array<bool, 1, Eigen::Dynamic>& gated_measurements,
    std::vector<Eigen::Index>& consumed_indices_out) const {
    std::vector<Eigen::Quaterniond> quats;
    quats.reserve(gated_measurements.size());

    for (Eigen::Index k = 0;
         k < static_cast<Eigen::Index>(angular_gate_indices.size()); ++k) {
        if (gated_measurements[k]) {
            Eigen::Index original_idx = angular_gate_indices[k];
            quats.push_back(measurements[original_idx].ori_quaternion());
            consumed_indices_out.push_back(original_idx);
        }
    }
    return quats;
}

void PoseTrackManager::update_track_orientation(
    Track& track,
    const std::vector<Eigen::Quaterniond>& quaternions) {
    if (quaternions.empty()) {
        return;
    }

    Eigen::Quaterniond avg_q =
        vortex::utils::math::average_quaternions(quaternions);

    double d_prev = track.prev_orientation.angularDistance(avg_q);

    double d_curr = track.current_orientation.angularDistance(avg_q);

    constexpr double eps = 1e-6;

    double alpha = d_prev / (d_curr + eps);
    alpha = std::clamp(alpha, 0.05, 1.0);

    track.prev_orientation = track.current_orientation;

    track.current_orientation = track.current_orientation.slerp(alpha, avg_q);
}

void PoseTrackManager::erase_gated_measurements(
    std::vector<Pose>& measurements,
    std::vector<Eigen::Index>& indices) {
    std::ranges::sort(indices, std::greater<>());
    for (Eigen::Index idx : indices) {
        measurements.erase(measurements.begin() + idx);
    }
}

void PoseTrackManager::create_tracks(const std::vector<Pose>& measurements) {
    tracks_.reserve(tracks_.size() + measurements.size());

    auto make_track = [this](const Pose& measurement) {
        return Track{.id = track_id_counter_++,
                     .state = vortex::prob::Gauss3d(
                         measurement.pos_vector(),
                         Eigen::Matrix<double, 3, 3>::Identity()),
                     .current_orientation = measurement.ori_quaternion(),
                     .prev_orientation = measurement.ori_quaternion(),
                     .existence_probability =
                         existence_config_.initial_existence_probability,
                     .confirmed = false};
    };

    for (const Pose& m : measurements) {
        tracks_.push_back(make_track(m));
    }
}

void PoseTrackManager::confirm_tracks() {
    for (Track& track : tracks_) {
        if (!track.confirmed && track.existence_probability >=
                                    existence_config_.confirmation_threshold) {
            track.confirmed = true;
        }
    }
}

void PoseTrackManager::delete_tracks() {
    auto new_end = std::ranges::remove_if(tracks_, [this](Track& track) {
        return track.existence_probability <
               existence_config_.deletion_threshold;
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

#include "line_filtering/lib/line_track_manager.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

namespace vortex::line_filtering {

LineTrackManager::LineTrackManager(const LineTrackManagerConfig& config)
    : track_id_counter_(0), config_(config) {}

void LineTrackManager::step(std::vector<LineMeasurement>& measurements,
                            double dt) {
    sort_tracks_by_priority();

    for (LineTrack& track : tracks_) {
        const auto& cfg = config_.default_config;

        Eigen::Array<double, 2, Eigen::Dynamic> meas_residuals =
            compute_residuals(track, measurements);

        LineGate2D gate;
        gate.min_rho_error = cfg.min_rho_error;
        gate.max_rho_error = cfg.max_rho_error;
        gate.min_phi_error = cfg.min_phi_error;
        gate.max_phi_error = cfg.max_phi_error;
        gate.mahalanobis_threshold = cfg.mahalanobis_threshold;

        DynMod dyn_mod(cfg.dyn_std_dev);
        SensorMod sensor_mod(cfg.sens_std_dev);

        vortex::filter::IPDA<DynMod, SensorMod>::Config ipda_cfg;
        ipda_cfg.ipda.estimate_clutter = cfg.estimate_clutter;
        ipda_cfg.ipda.prob_of_survival = cfg.prob_of_survival;
        ipda_cfg.pdaf.mahalanobis_threshold = cfg.mahalanobis_threshold;
        ipda_cfg.pdaf.prob_of_detection = cfg.prob_of_detection;
        ipda_cfg.pdaf.clutter_intensity = cfg.clutter_intensity;

        const IPDA::State state_prev{
            .x_estimate = track.error_state,
            .existence_probability = track.existence_probability};

        auto result = IPDA::step(dyn_mod, sensor_mod, dt, state_prev,
                                 meas_residuals, ipda_cfg, gate);

        track.error_state = result.state.x_estimate;
        track.existence_probability = result.state.existence_probability;
        inject_and_reset(track);

        erase_gated_measurements(measurements, result.gated_measurements);
    }

    confirm_tracks();
    delete_tracks();
    create_tracks(measurements);
}

Eigen::Vector2d LineTrackManager::log_residual(const NominalLine& pred,
                                               LineMeasurement meas) {
    if (pred.n.dot(meas.n) < 0.0) {
        // Sign alignment: if normals point in opposite half-planes, flip
        meas.n = -meas.n;
        meas.rho = -meas.rho;
    }

    const double dot = pred.n.dot(meas.n);
    const double cross = pred.n.x() * meas.n.y() - pred.n.y() * meas.n.x();
    const double delta_phi = std::atan2(cross, dot);  // (-pi, pi]

    const double delta_rho = meas.rho - pred.rho;

    return Eigen::Vector2d{delta_rho, delta_phi};
}

void LineTrackManager::inject_and_reset(LineTrack& track) {
    const Eigen::Vector2d delta = track.error_state.mean();

    track.nominal.rho += delta(0);

    const double dphi = delta(1);
    const double c = std::cos(dphi);
    const double s = std::sin(dphi);
    Eigen::Vector2d n_old = track.nominal.n;
    track.nominal.n = Eigen::Vector2d{c * n_old.x() - s * n_old.y(),
                                      s * n_old.x() + c * n_old.y()};
    track.nominal.n.normalize();

    // First-order reset Jacobian:
    //   G = [ 1  0 ]
    //       [ 0  1 - 0 ]  (for SO(2) the correction is trivial; G ≈ I)
    // For a 2D rotation the first-order correction is exactly identity.
    // (No skew-symmetric correction needed in 2D.)

    track.error_state.mean().setZero();
}

Eigen::Array<double, 2, Eigen::Dynamic> LineTrackManager::compute_residuals(
    const LineTrack& track,
    const std::vector<LineMeasurement>& measurements) const {
    IPDA::Arr_zXd meas_residuals(2, measurements.size());

    for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(measurements.size());
         ++k) {
        meas_residuals.matrix().col(k) =
            log_residual(track.nominal, measurements[k]);
    }

    return meas_residuals;
}

void LineTrackManager::erase_gated_measurements(
    std::vector<LineMeasurement>& measurements,
    const Eigen::Array<bool, 1, Eigen::Dynamic>& mask) const {
    for (Eigen::Index k = mask.cols() - 1; k >= 0; --k) {
        if (mask(k)) {
            measurements.erase(measurements.begin() + k);
        }
    }
}

void LineTrackManager::create_tracks(
    const std::vector<LineMeasurement>& measurements) {
    tracks_.reserve(tracks_.size() + measurements.size());
    const auto& cfg = config_.default_config;

    for (const auto& m : measurements) {
        Eigen::Matrix2d P0 = Eigen::Matrix2d::Zero();
        P0(0, 0) = cfg.init_rho_std * cfg.init_rho_std;
        P0(1, 1) = cfg.init_phi_std * cfg.init_phi_std;

        tracks_.push_back(LineTrack{
            .id = track_id_counter_++,
            .nominal = NominalLine{.rho = m.rho, .n = m.n},
            .error_state = State2d(Eigen::Vector2d::Zero(), P0),
            .existence_probability =
                config_.existence.initial_existence_probability,
            .confirmed = false,
        });
    }
}

void LineTrackManager::confirm_tracks() {
    for (LineTrack& t : tracks_) {
        if (!t.confirmed && t.existence_probability >=
                                config_.existence.confirmation_threshold) {
            t.confirmed = true;
        }
    }
}

void LineTrackManager::delete_tracks() {
    auto new_end = std::ranges::remove_if(tracks_, [this](const LineTrack& t) {
        return t.existence_probability < config_.existence.deletion_threshold;
    });
    tracks_.erase(new_end.begin(), new_end.end());
}

void LineTrackManager::sort_tracks_by_priority() {
    std::ranges::sort(tracks_, [](const LineTrack& a, const LineTrack& b) {
        if (a.confirmed != b.confirmed)
            return a.confirmed > b.confirmed;
        return a.existence_probability > b.existence_probability;
    });
}

}  // namespace vortex::line_filtering

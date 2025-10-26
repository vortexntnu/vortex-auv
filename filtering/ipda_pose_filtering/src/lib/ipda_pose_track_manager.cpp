#include "ipda_pose_filtering/lib/ipda_pose_track_manager.hpp"
#include <algorithm>
#include <ranges>

namespace vortex::filtering {

IPDAPoseTrackManager::IPDAPoseTrackManager(const TrackManagerConfig& config)
    : track_id_counter_(0),
      dyn_mod_(config.dyn_mod.std_pos, config.dyn_mod.std_orient),
      sensor_mod_(config.sensor_mod.std_pos,
                  config.sensor_mod.std_orient,
                  config.sensor_mod.max_angle_gate_threshold) {
    ipda_config_ = config.ipda;
    existence_config_ = config.existence;
}

void IPDAPoseTrackManager::predict_tracks() {
    auto new_end = std::ranges::remove_if(tracks_, [this](Track& track) {
      track.existence_probability = IPDA::existence_prediction(
        track.existence_probability, ipda_config_.ipda.prob_of_survival);

      return track.existence_probability < existence_config_.deletion_threshold;
    });
    tracks_.erase(new_end.begin(), new_end.end());
}

void IPDAPoseTrackManager::measurement_update(Measurements& measurements, double dt){
      std::ranges::sort(tracks_, [](const Track& a, const Track& b) {
        if (a.confirmed != b.confirmed)
            return a.confirmed > b.confirmed;
        return a.existence_probability > b.existence_probability;
    });

    for (Track &track : tracks_){
      const IPDA::State state_est_prev{
        .x_estimate = track.state,
        .existence_probability = track.existence_probability
      };

      auto output = IPDA::step(dyn_mod_, sensor_mod_, dt,
        state_est_prev, measurements, ipda_config_);

      track.state = output.state.x_estimate;
      track.existence_probability = output.state.existence_probability;

      std::vector<Eigen::Vector6d> outside_cols;
      outside_cols.reserve(measurements.cols());
      for (Eigen::Index i = 0; i < measurements.cols(); ++i)
          if (!output.gated_measurements[i])
              outside_cols.push_back(measurements.col(i));

      if (!outside_cols.empty()) {
          Measurements outside(6, static_cast<Eigen::Index>(outside_cols.size()));
          for (Eigen::Index i = 0; i < outside.cols(); ++i)
              outside.col(i) = outside_cols[i];
          measurements = std::move(outside);
      }
    }
    create_tracks(measurements);
}


void IPDAPoseTrackManager::create_tracks(const Measurements& measurements) {
  tracks_.reserve(tracks_.size() + measurements.cols());
  for(auto i : std::views::iota(Eigen::Index{0}, measurements.cols())) {
    tracks_.emplace_back(
      Track{
      .id = track_id_counter_++,
      .state = vortex::prob::Gauss6d(measurements.col(i), Eigen::Matrix<double, 6, 6>::Identity()),
      .existence_probability = existence_config_.initial_existence_probability,
      .confirmed = false
      });
  }
}

}  // namespace vortex::filtering

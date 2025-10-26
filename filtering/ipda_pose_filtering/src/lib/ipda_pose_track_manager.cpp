#include "ipda_pose_filtering/lib/ipda_pose_track_manager.hpp"

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

void IPDAPoseTrackManager::predict_tracks(const Measurements& measurements,
                                          double dt) {}

}  // namespace vortex::filtering

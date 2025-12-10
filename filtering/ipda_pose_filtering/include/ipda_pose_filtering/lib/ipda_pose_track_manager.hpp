#ifndef IPDA_POSE_TRACK_MANAGER_HPP
#define IPDA_POSE_TRACK_MANAGER_HPP

#include <memory>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include "typedefs.hpp"

namespace vortex::filtering {

class IPDAPoseTrackManager {
   public:
    IPDAPoseTrackManager(const TrackManagerConfig& config);

    void predict_tracks();

    void measurement_update(Measurements& measurements, double dt);

    const std::vector<Track>& get_tracks() { return tracks_; }

   private:
    void create_tracks(const Measurements& measurements);

    int track_id_counter_ = 0;

    std::vector<Track> tracks_;

    DynMod dyn_mod_;

    SensorMod sensor_mod_;

    vortex::filter::IPDA<DynMod, SensorMod>::Config ipda_config_;

    ExistenceManagementConfig existence_config_;
};

}  // namespace vortex::filtering

#endif  // IPDA_POSE_TRACK_MANAGER_HPP

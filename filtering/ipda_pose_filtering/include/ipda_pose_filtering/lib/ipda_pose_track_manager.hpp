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

    void step(std::vector<Pose>& measurements, double dt);

    const std::vector<Track>& get_tracks() { return tracks_; }

   private:
    std::vector<Eigen::Index> angular_gate_measurements(
        const Track& track,
        const std::vector<Pose>& measurements) const;

    void delete_tracks();

    void create_tracks(const std::vector<Pose>& measurements);

    void sort_tracks_by_priority();

    Eigen::Matrix<double, 3, Eigen::Dynamic> build_position_matrix(
        const std::vector<Pose>& measurements,
        const std::vector<Eigen::Index>& indices) const;

    std::vector<Eigen::Quaterniond> collect_used_quaternions(
        const std::vector<Pose>& measurements,
        const std::vector<Eigen::Index>& angular_gate_indices,
        const Eigen::Array<bool, 1, Eigen::Dynamic>& gated_measurements,
        std::vector<Eigen::Index>& consumed_indices_out) const;

    void update_track_orientation(
        Track& track,
        const std::vector<Eigen::Quaterniond>& quaternions);

    void erase_gated_measurements(std::vector<Pose>& measurements,
                                  std::vector<Eigen::Index>& indices);

    int track_id_counter_ = 0;

    std::vector<Track> tracks_;

    DynMod dyn_mod_;

    SensorMod sensor_mod_;

    vortex::filter::IPDA<DynMod, SensorMod>::Config ipda_config_;

    ExistenceManagementConfig existence_config_;

    double max_angle_gate_threshold_{};
};

}  // namespace vortex::filtering

#endif  // IPDA_POSE_TRACK_MANAGER_HPP

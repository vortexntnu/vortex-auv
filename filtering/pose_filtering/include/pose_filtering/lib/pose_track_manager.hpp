#ifndef POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_
#define POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_

#include <memory>
#include <vector>
#include <vortex_filtering/vortex_filtering.hpp>
#include "typedefs.hpp"

namespace vortex::filtering {

/**
 * @file pose_track_manager.hpp
 * @brief Track management utilities for pose-based measurements.
 *
 * The PoseTrackManager implements track creation, confirmation,
 * deletion and update logic using an IPDA filter for each track. It
 * handles spatial and angular gating for associating pose measurements
 * to existing tracks.
 */

/**
 * @brief Class responsible for maintaining a set of tracks based on
 * pose measurements.
 *
 * Public API:
 *  - construct with a TrackManagerConfig
 *  - call step() each frame with a list of measurements and the time step
 *  - retrieve current tracks with get_tracks()
 */
class PoseTrackManager {
   public:
    /**
     * @brief Construct a PoseTrackManager
     * @param config Configuration parameters for IPDA, models and existence
     * management
     */
    explicit PoseTrackManager(const TrackManagerConfig& config);

    /**
     * @brief Perform a single tracking step: associate measurements, update
     * filters and manage track lifecycle.
     * @param measurements Vector of pose measurements (will be modified by
     * the manager to remove consumed measurements)
     * @param dt Time step in seconds
     */
    void step(std::vector<Pose>& measurements, double dt);

    /**
     * @brief Get the list of currently maintained tracks.
     * @return const reference to internal track vector
     */
    const std::vector<Track>& get_tracks() { return tracks_; }

   private:
    /**
     * @brief Compute angular gating for a given track against measurements.
     * @param track Track to gate
     * @param measurements Candidate measurements
     * @return Indices of measurements passing the angular gate
     */
    std::vector<Eigen::Index> angular_gate_measurements(
        const Track& track,
        const std::vector<Pose>& measurements) const;

    /**
     * @brief Confirm tracks which have exceeded the confirmation threshold.
     */
    void confirm_tracks();

    /**
     * @brief Delete tracks which have fallen below the deletion threshold.
     */
    void delete_tracks();

    /**
     * @brief Create new tracks from unassociated measurements.
     * @param measurements Measurements to be used for new track creation
     */
    void create_tracks(const std::vector<Pose>& measurements);

    /**
     * @brief Sort tracks by priority to determine processing / deletion order.
     */
    void sort_tracks_by_priority();

    /**
     * @brief Build a matrix of 3D positions for a subset of measurements.
     * @param measurements All measurements
     * @param indices Indices selecting which measurements to include
     * @return 3xN Eigen matrix with positions in columns
     */
    Eigen::Matrix<double, 3, Eigen::Dynamic> build_position_matrix(
        const std::vector<Pose>& measurements,
        const std::vector<Eigen::Index>& indices) const;

    /**
     * @brief Collect quaternions from gated measurements and mark consumed
     * indices.
     * @param measurements All measurements
     * @param angular_gate_indices Candidate indices that passed angular gate
     * @param gated_measurements Boolean mask of gated measurements
     * @param consumed_indices_out Output vector filled with indices consumed
     * by this collection
     * @return Vector of quaternions corresponding to consumed measurements
     */
    std::vector<Eigen::Quaterniond> collect_used_quaternions(
        const std::vector<Pose>& measurements,
        const std::vector<Eigen::Index>& angular_gate_indices,
        const Eigen::Array<bool, 1, Eigen::Dynamic>& gated_measurements,
        std::vector<Eigen::Index>& consumed_indices_out) const;

    /**
     * @brief Update a track's orientation using the collected quaternions.
     * @param track Track to update
     * @param quaternions Vector of quaternions to fuse / apply
     */
    void update_track_orientation(
        Track& track,
        const std::vector<Eigen::Quaterniond>& quaternions);

    /**
     * @brief Erase measurements which have been gated / consumed.
     * @param measurements Measurements vector (modified in place)
     * @param indices Indices of measurements to erase
     */
    void erase_gated_measurements(std::vector<Pose>& measurements,
                                  std::vector<Eigen::Index>& indices);

    // Internal bookkeeping
    int track_id_counter_ = 0;

    std::vector<Track> tracks_;

    DynMod dyn_mod_;

    SensorMod sensor_mod_;

    vortex::filter::IPDA<DynMod, SensorMod>::Config ipda_config_;

    ExistenceManagementConfig existence_config_;

    double max_angle_gate_threshold_{};
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_

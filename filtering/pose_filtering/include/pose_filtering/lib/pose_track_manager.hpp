#ifndef POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_
#define POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_

#include <cstdint>
#include <limits>
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

struct PoseGate6D {
    double min_pos_error = 0.0;
    double max_pos_error = std::numeric_limits<double>::infinity();
    double min_ori_error = 0.0;
    double max_ori_error = std::numeric_limits<double>::infinity();

    double mahalanobis_threshold = std::numeric_limits<double>::infinity();

    using Vec_z = IPDA::Vec_z;
    using Gauss_z = IPDA::Gauss_z;

    bool operator()(const Vec_z& z, const Gauss_z& z_pred) const {
        const Vec_z r = z - z_pred.mean();
        const double pos = r.template head<3>().norm();
        const double ori = r.template tail<3>().norm();

        if (pos > max_pos_error || ori > max_ori_error) {
            return false;
        }

        if (pos <= min_pos_error && ori <= min_ori_error) {
            return true;
        }

        return z_pred.mahalanobis_distance(z) <= mahalanobis_threshold;
    }
};

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
    void step(std::vector<Landmark>& measurements, double dt);

    /**
     * @brief Get the list of currently maintained tracks.
     * @return const reference to internal track vector
     */
    const std::vector<Track>& get_tracks() const { return tracks_; }

    /**
     * @brief Return whether we have a track of the specified type and subtype.
     * @param type of the track
     * @param subtype of the track
     * @return Whether at least one track with the specified type and subtype
     * exists.
     */
    bool has_track(uint16_t type, uint16_t subtype) const {
        return std::any_of(tracks_.begin(), tracks_.end(), [&](const Track& t) {
            return t.confirmed && t.type == type && t.subtype == subtype;
        });
    }

    bool has_track(int id) const {
        return std::any_of(tracks_.begin(), tracks_.end(), [&](const Track& t) {
            return t.confirmed && t.id == id;
        });
    }

    std::vector<const Track*> get_tracks_by_type(uint16_t type,
                                                 uint16_t subtype) const {
        std::vector<const Track*> out;
        out.reserve(tracks_.size());
        for (const auto& t : tracks_) {
            if (!(t.type == type && t.subtype == subtype && !t.confirmed)) {
                continue;
            }
            out.push_back(&t);
        }
        return out;
    }

   private:
    std::vector<Eigen::Index> type_gate_measurements(
        const Track& track,
        const std::vector<Landmark>& measurements) const;

    Eigen::Array<double, 6, Eigen::Dynamic> compute_measurement_residuals(
        const Track& track,
        const std::vector<Landmark>& measurements,
        const std::vector<Eigen::Index>& indices) const;

    /**
     * @brief Map a quaternion to the so(3) tangent vector (log map).
     * @param q_in Quaternion representing the relative rotation
     * @return Vector in R^3 representing the axis * angle
     */
    Eigen::Vector3d so3_log_quat(const Eigen::Quaterniond& q_in) const;

    void inject_and_reset(Track& track);

    /**
     * @brief Map a tangent vector back to a quaternion (exp map).
     * @param rvec Tangent vector (axis * angle)
     * @return Quaternion corresponding to the rotation
     */
    Eigen::Quaterniond so3_exp_quat(const Eigen::Vector3d& rvec) const;

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
    void create_tracks(const std::vector<Landmark>& measurements);

    /**
     * @brief Sort tracks by priority to determine processing / deletion order.
     */
    void sort_tracks_by_priority();

    /**
     * @brief Erase measurements which have been gated / consumed.
     * @param measurements Measurements vector (modified in place)
     * @param indices Indices of measurements to erase
     */
    void erase_gated_measurements(
        std::vector<Landmark>& measurements,
        const std::vector<Eigen::Index>& global_indices,
        const Eigen::Array<bool, 1, Eigen::Dynamic>& mask) const;

    const LandmarkClassConfig& cfg_for(uint16_t type, uint16_t subtype) const {
        for (const auto& [k, cfg] : config_.per_class_configs) {
            if (k.type == type && k.subtype == subtype)
                return cfg;
        }
        return config_.default_class_config;
    }

    const LandmarkClassConfig& cfg_for(const Track& t) const {
        return cfg_for(t.type, t.subtype);
    }

    const LandmarkClassConfig& cfg_for(const Landmark& m) const {
        return cfg_for(m.class_key.type, m.class_key.subtype);
    }

    // Internal bookkeeping
    int track_id_counter_{0};

    std::vector<Track> tracks_;

    TrackManagerConfig config_;
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_

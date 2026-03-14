#ifndef POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_
#define POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_

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
 * deletion and update logic using a PDAF filter for each track with
 * N/M logic for track lifecycle management. It handles spatial and
 * angular gating for associating pose measurements to existing tracks.
 */

/**
 * @brief Struct representing a 6D pose gating mechanism.
 *
 * This struct is used to determine whether a given pose measurement
 * falls within acceptable positional and orientational error bounds.
 */
struct PoseGate6D {
    double min_pos_error = 0.0;
    double max_pos_error = std::numeric_limits<double>::infinity();
    double min_ori_error = 0.0;
    double max_ori_error = std::numeric_limits<double>::infinity();

    double mahalanobis_threshold = std::numeric_limits<double>::infinity();

    using Vec_z = PDAF::Vec_z;
    using Gauss_z = PDAF::Gauss_z;

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
     * @param config Configuration parameters for PDAF, models and N/M
     * track management
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
     * @param class_key LandmarkClassKey to match
     * @return Whether at least one confirmed track with the specified class key
     * exists.
     */
    bool has_track(const LandmarkClassKey& class_key) const {
        return std::any_of(tracks_.begin(), tracks_.end(), [&](const Track& t) {
            return t.confirmed && t.class_key == class_key;
        });
    }

    /**
     * @brief Return whether we have a track with the specified ID.
     * @param id of the track
     * @return Whether at least one track with the specified ID exists.
     */
    bool has_track(int id) const {
        return std::any_of(tracks_.begin(), tracks_.end(), [&](const Track& t) {
            return t.confirmed && t.id == id;
        });
    }

    /**
     * @brief Get all confirmed tracks of the specified class key.
     * @param class_key LandmarkClassKey to match
     * @return Vector of pointers to tracks matching the specified class key.
     */
    std::vector<const Track*> get_tracks_by_type(
        const LandmarkClassKey& class_key) const {
        std::vector<const Track*> out;
        out.reserve(tracks_.size());
        for (const auto& t : tracks_) {
            if (t.class_key == class_key && t.confirmed) {
                out.push_back(&t);
            }
        }
        return out;
    }

   private:
    /**
     * @brief Gate measurements for a track based on class key and gating
     * parameters.
     * @param track Track for which to gate measurements
     * @param measurements Vector of measurements to gate against
     * @return Vector of indices of measurements that passed the gate
     */
    std::vector<Eigen::Index> gate_measurements_by_class(
        const Track& track,
        const std::vector<Landmark>& measurements) const;

    /**
     * @brief Compute measurement residuals for a track against a set of
     * measurements.
     * @param track Track for which to compute residuals
     * @param measurements Vector of measurements to compute residuals against
     * @param indices Indices of measurements to use from the measurements
     * vector
     * @return 6xN array of residuals (position and orientation) for each
     * measurement
     */
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

    /**
     * @brief Inject the error state into the nominal state and reset the error
     * state.
     * @param track Track for which to perform the injection and reset
     */
    void inject_and_reset(Track& track);

    /**
     * @brief Map a tangent vector back to a quaternion (exp map).
     * @param rvec Tangent vector (axis * angle)
     * @return Quaternion corresponding to the rotation
     */
    Eigen::Quaterniond so3_exp_quat(const Eigen::Vector3d& rvec) const;

    /**
     * @brief Record a hit or miss for a track and update its history window.
     * @param track Track to update
     * @param hit Whether the track was associated with a measurement
     */
    void record_hit_miss(Track& track, bool hit);

    /**
     * @brief Confirm tracks which meet the N/M confirmation criteria.
     */
    void confirm_tracks();

    /**
     * @brief Delete tracks which meet the N/M deletion criteria.
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

    /**
     * @brief Get the configuration for a specific landmark class key.
     * @param key Landmark class key
     * @return Configuration for the specified class key
     */
    const LandmarkClassConfig& cfg_for(const LandmarkClassKey& key) const {
        for (const auto& [k, cfg] : config_.per_class_configs) {
            if (k == key)
                return cfg;
        }
        return config_.default_class_config;
    }

    /**
     * @brief Get the configuration for a specific track.
     * @param t Track for which to get the configuration
     * @return Configuration for the specified track
     */
    const LandmarkClassConfig& cfg_for(const Track& t) const {
        return cfg_for(t.class_key);
    }

    /**
     * @brief Get the configuration for a specific landmark.
     * @param m Landmark for which to get the configuration
     * @return Configuration for the specified landmark
     */
    const LandmarkClassConfig& cfg_for(const Landmark& m) const {
        return cfg_for(m.class_key);
    }

    /**
     * @brief Validate the configuration parameters. Throw on failure.
     * @param config Configuration to validate
     */
    void validate_config(const TrackManagerConfig& config) {
        const auto validate_nm = [](const NMConfig& nm) {
            if (nm.confirm_n < 0 || nm.confirm_m <= 0 || nm.delete_n < 0 ||
                nm.delete_m <= 0) {
                throw std::invalid_argument(
                    "NMConfig values must be non-negative, and *_m must be > "
                    "0");
            }
            if (nm.confirm_n > nm.confirm_m) {
                throw std::invalid_argument("confirm_n must be <= confirm_m");
            }
            if (nm.delete_n > nm.delete_m) {
                throw std::invalid_argument("delete_n must be <= delete_m");
            }
        };

        validate_nm(config.default_class_config.nm);
        for (const auto& [key, class_cfg] : config.per_class_configs) {
            validate_nm(class_cfg.nm);
        }
    }

    // Internal bookkeeping
    int track_id_counter_{0};

    std::vector<Track> tracks_;

    TrackManagerConfig config_;
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__POSE_TRACK_MANAGER_HPP_

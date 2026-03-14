#ifndef POSE_FILTERING__LIB__TYPEDEFS_HPP_
#define POSE_FILTERING__LIB__TYPEDEFS_HPP_

#include <deque>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <vector>
#include <vortex/utils/types.hpp>
#include <vortex_filtering/filters/pdaf.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

namespace vortex::filtering {

/**
 * @file typedefs.hpp
 * @brief Common type aliases and configuration structs used by the
 * pose_filtering library.
 *
 * This header exposes convenient using-aliases for commonly used types
 * (state, dynamic and sensor models, PDAF filter instance, and Pose type)
 * and configuration structs that are passed to the track manager.
 */

/**
 * @brief Gaussian 6d state representation (mean + covariance).
 */
using State6d = vortex::prob::Gauss6d;

/**
 * @brief Discrete-time constant dynamic model of order 6.
 */
using DynMod = vortex::models::ConstantDynamicModel<6>;

/**
 * @brief Identity sensor model mapping state to measurement space.
 */
using SensorMod = vortex::models::IdentitySensorModel<6, 6>;

/**
 * @brief PDAF filter type specialized for the chosen dynamic and sensor models.
 */
using PDAF = vortex::filter::PDAF<DynMod, SensorMod>;

/**
 * @brief Pose type (position + quaternion) from vortex utils.
 */
using Pose = vortex::utils::types::Pose;

/**
 * @brief N/M logic parameters for track confirmation and deletion.
 *
 * Confirmation: track is confirmed when it accumulates at least
 * confirm_n hits in the last confirm_m steps.
 * Deletion: track is deleted when it accumulates at least
 * delete_n misses in the last delete_m steps.
 */
struct NMConfig {
    int confirm_n{3};
    int confirm_m{5};
    int delete_n{5};
    int delete_m{7};
};

struct LandmarkClassKey {
    uint16_t type{};
    uint16_t subtype{};

    bool operator==(const LandmarkClassKey&) const = default;
};

struct LandmarkClassConfig {
    // N/M track lifecycle parameters
    NMConfig nm;

    // noise (simple version: one std_dev for all 6 dims)
    double dyn_std_dev = 1.0;
    double sens_std_dev = 1.0;

    // initial cov (separate pos/orientation)
    double init_pos_std = 0.1;
    double init_ori_std = 0.05;

    // gating thresholds (your min-pass + hard max + mahal)
    double min_pos_error = 0.0;  // meters
    double max_pos_error = 1.0;  // meters
    double min_ori_error = 0.0;  // radians
    double max_ori_error = 0.5;  // radians

    double mahalanobis_threshold = 2.5;
    double prob_of_detection = 0.5;
    double clutter_intensity = 0.01;
};

/**
 * @brief High-level track manager configuration struct.
 *
 * Contains PDAF-specific configuration and parameters for dynamics,
 * sensor model, N/M track management and gating thresholds.
 */
struct TrackManagerConfig {
    LandmarkClassConfig default_class_config;
    std::vector<std::pair<LandmarkClassKey, LandmarkClassConfig>>
        per_class_configs;
};

/**
 * @brief Struct representing the error state of a track (position and
 * orientation errors in local tangent space).
 */
struct ErrorState {
    Eigen::Vector3d pos_error;
    Eigen::Vector3d ori_error;
};

/**
 * @brief Struct representing the nominal state of a track (position and
 * orientation).
 */
struct NominalState {
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
};

struct Landmark {
    vortex::utils::types::Pose pose{};
    LandmarkClassKey class_key{};
};

/**
 * @brief Representation of a single track maintained by the track manager.
 *
 * The Track stores the filter state, orientation quaternions and bookkeeping
 * fields used for existence management and confirmation.
 */
struct Track {
    /// Unique track identifier
    int id{};

    /// Landmark class (type + subtype)
    LandmarkClassKey class_key{};

    /// Nominal state representation (position and orientation)
    NominalState nominal_state;

    /// Error state representation (position and orientation errors)
    State6d error_state;

    /// Whether the track has been confirmed (passed N/M confirmation)
    bool confirmed{false};

    /// Sliding window of hit/miss history (true = hit, false = miss)
    std::deque<bool> hit_history{};

    /// Number of hits in the current window
    int hits() const {
        return static_cast<int>(
            std::count(hit_history.begin(), hit_history.end(), true));
    }

    /// Number of misses in the current window
    int misses() const { return static_cast<int>(hit_history.size()) - hits(); }

    /**
     * @brief Convert the track position + orientation to a Pose object.
     * @return vortex::utils::types::Pose constructed from the state mean and
     * current_orientation
     */
    Pose to_pose() const {
        return vortex::utils::types::Pose::from_eigen(nominal_state.pos,
                                                      nominal_state.ori);
    }
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__TYPEDEFS_HPP_

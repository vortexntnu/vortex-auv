#ifndef POSE_FILTERING__LIB__TYPEDEFS_HPP_
#define POSE_FILTERING__LIB__TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>
#include <vortex_filtering/filters/ipda.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

namespace vortex::filtering {

/**
 * @file typedefs.hpp
 * @brief Common type aliases and configuration structs used by the
 * pose_filtering library.
 *
 * This header exposes convenient using-aliases for commonly used types
 * (state, dynamic and sensor models, IPDA filter instance, and Pose type)
 * and configuration structs that are passed to the track manager.
 */

/**
 * @brief Gaussian state representation for 3D pose (mean + covariance).
 */
using State3d = vortex::prob::Gauss3d;

/**
 * @brief Discrete-time constant dynamic model of order 3.
 */
using DynMod = vortex::models::ConstantDynamicModel<3>;

/**
 * @brief Identity sensor model mapping state to measurement space.
 */
using SensorMod = vortex::models::IdentitySensorModel<3, 3>;

/**
 * @brief IPDA filter type specialized for the chosen dynamic and sensor models.
 */
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

/**
 * @brief Pose type (position + quaternion) from vortex utils.
 */
using Pose = vortex::utils::types::Pose;

/**
 * @brief Configuration for the dynamic model (process noise standard
 * deviation).
 */
struct DynModConfig {
    double std_dev{1.0};
};

/**
 * @brief Configuration for the sensor model (measurement noise standard
 * deviation).
 */
struct SensorModConfig {
    double std_dev{1.0};
};

/**
 * @brief Parameters for existence management (track confirmation / deletion).
 */
struct ExistenceManagementConfig {
    double confirmation_threshold{0.6};
    double deletion_threshold{0.2};
    double initial_existence_probability{0.5};
};

/**
 * @brief High-level track manager configuration struct.
 *
 * Contains IPDA-specific configuration and parameters for dynamics,
 * sensor model, existence management and gating thresholds.
 */
struct TrackManagerConfig {
    vortex::filter::IPDA<DynMod, SensorMod>::Config ipda;
    DynModConfig dyn_mod;
    SensorModConfig sensor_mod;
    ExistenceManagementConfig existence;
    double max_angle_gate_threshold{};
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

    /// Filter state (mean + covariance)
    State3d state;

    /// Current orientation as quaternion (w, x, y, z)
    Eigen::Quaterniond current_orientation;

    /// Previous orientation (useful for angular gating / smoothing)
    Eigen::Quaterniond prev_orientation;

    /// Probability that the track exists (0..1)
    double existence_probability{0.0};

    /// Whether the track has been confirmed (passed confirmation threshold)
    bool confirmed{false};

    /**
     * @brief Convert the filter state + orientation to a Pose object.
     * @return vortex::utils::types::Pose constructed from the state mean and
     * current_orientation
     */
    Pose to_pose() const {
        return vortex::utils::types::Pose::from_eigen(state.mean(),
                                                      current_orientation);
    }

    /**
     * @brief Compute angular distance between the track orientation and a
     * measurement pose orientation.
     * @param z Measurement pose to compare against
     * @return Angular distance in radians (double)
     */
    double angular_distance(const Pose& z) const {
        return current_orientation.angularDistance(z.ori_quaternion());
    }
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__TYPEDEFS_HPP_

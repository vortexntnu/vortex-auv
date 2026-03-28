/**
 * @file adaptive_los.hpp
 * @brief Defines the AdaptiveLOSGuidance class and its parameter structure for
 * the adaptive LOS guidance algorithm.
 */

#ifndef LOS_GUIDANCE__LIB__ADAPTIVE_LOS_HPP_
#define LOS_GUIDANCE__LIB__ADAPTIVE_LOS_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

/**
 * @brief Parameters for the Adaptive LOS guidance algorithm.
 *
 * This structure stores the tuning parameters used by the adaptive LOS
 * controller for horizontal and vertical path-following.
 */
struct AdaptiveLosParams {
    double lookahead_distance_h{};
    double lookahead_distance_v{};
    double adaptation_gain_h{};
    double adaptation_gain_v{};
    double time_step{};  // in milliseconds
};

/**
 * @brief Implements the adaptive LOS guidance algorithm.
 *
 * This class computes desired heading and pitch commands based on the current
 * vehicle position and the active path segment.
 */
class AdaptiveLOSGuidance {
   public:
    /**
     * @brief Constructs an AdaptiveLOSGuidance object.
     * @param params Parameters for the adaptive LOS guidance algorithm.
     */
    explicit AdaptiveLOSGuidance(const AdaptiveLosParams& params);

    /**
     * @brief Destroys the AdaptiveLOSGuidance object.
     */
    ~AdaptiveLOSGuidance() = default;

    /**
     * @brief Calculates the desired LOS guidance outputs.
     * @param inputs Input values required for adaptive LOS computation.
     * @return types::Outputs The desired heading and pitch commands.
     */
    types::Outputs calculate_outputs(const types::Inputs& inputs);

   private:
    /**
     * @brief Updates the path heading and path pitch angles from the active
     * path segment.
     * @param inputs Input values containing the previous and next path points.
     */
    void update_angles(const types::Inputs& inputs);

    /**
     * @brief Calculates the cross-track error in the path-fixed reference
     * frame.
     * @param inputs Input values containing the current vehicle position and
     * path information.
     * @return types::CrossTrackError The calculated horizontal and vertical
     * cross-track errors.
     */
    const types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs);

    /**
     * @brief Updates the adaptive estimates based on the current cross-track
     * error.
     * @param cross_track_error The current cross-track error in the path frame.
     */
    void update_adaptive_estimates(
        const types::CrossTrackError& cross_track_error);

    /**
     * @brief Parameters used by the adaptive LOS guidance algorithm.
     */
    AdaptiveLosParams params_{};

    /**
     * @brief Rotation matrix for the path pitch rotation about the y-axis.
     */
    Eigen::Matrix3d rotation_y_ = Eigen::Matrix3d::Zero();

    /**
     * @brief Rotation matrix for the path heading rotation about the z-axis.
     */
    Eigen::Matrix3d rotation_z_ = Eigen::Matrix3d::Zero();

    /**
     * @brief Stores the horizontal path angle.
     */
    double path_heading_{0.0};

    /**
     * @brief Stores the vertical path angle.
     */
    double path_pitch_{0.0};

    /**
     * @brief Stores the horizontal adaptive estimate.
     */
    double beta_c_hat_{0.0};

    /**
     * @brief Stores the vertical adaptive estimate.
     */
    double alpha_c_hat_{0.0};
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LIB__ADAPTIVE_LOS_HPP_

/**
 * @file integral_los.hpp
 * @brief Defines the IntegralLOSGuidance class and parameter structure for the
 * integral LOS guidance algorithm.
 */
#ifndef LOS_GUIDANCE__LIB__INTEGRAL_LOS_HPP_
#define LOS_GUIDANCE__LIB__INTEGRAL_LOS_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

/**
 * @brief Parameters for the Integral LOS guidance algorithm.
 */
struct IntegralLosParams {
    double proportional_gain_h{};
    double proportional_gain_v{};
    double integral_gain_h{};
    double integral_gain_v{};
    double time_step{};  // in milliseconds
};

/**
 * @brief Implements the integral LOS guidance algorithm.
 *
 * This class computes desired heading and pitch commands using proportional and
 * integral cross-track error feedback.
 */
class IntegralLOSGuidance {
   public:
    /**
     * @brief Constructs an IntegralLOSGuidance object.
     * @param params Parameters for the integral LOS guidance algorithm.
     */
    explicit IntegralLOSGuidance(const IntegralLosParams& params);

    /**
     * @brief Destroys the IntegralLOSGuidance object.
     */
    ~IntegralLOSGuidance() = default;

    /**
     * @brief Calculates the desired LOS guidance outputs.
     * @param inputs Input values required for integral LOS computation.
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
    types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs);

    /**
     * @brief Parameters used by the integral LOS guidance algorithm.
     */
    IntegralLosParams m_params{};

    /**
     * @brief Stores the integral of the horizontal cross-track error.
     */
    double integrated_horizontal_error_{};

    /**
     * @brief Stores the integral of the vertical cross-track error.
     */
    double integrated_vertical_error_{};

    /**
     * @brief Stores the horizontal path angle.
     */
    double path_heading_{};

    /**
     * @brief Stores the vertical path angle.
     */
    double path_pitch_{};

    /**
     * @brief Rotation representation for the path pitch angle about the y-axis.
     */
    Eigen::AngleAxisd rotation_y_{0.0, Eigen::Vector3d::UnitY()};

    /**
     * @brief Rotation representation for the path heading angle about the
     * z-axis.
     */
    Eigen::AngleAxisd rotation_z_{0.0, Eigen::Vector3d::UnitZ()};
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LIB__INTEGRAL_LOS_HPP_

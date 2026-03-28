/**
 * @file vector_field_los.hpp
 * @brief Defines the VectorFieldLOSGuidance class and parameter structure for
 * the vector field LOS guidance algorithm.
 */
#ifndef LOS_GUIDANCE__LIB__VECTOR_FIELD_LOS_HPP_
#define LOS_GUIDANCE__LIB__VECTOR_FIELD_LOS_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

/**
 * @brief Parameters for the vector field LOS guidance algorithm.
 */
struct VectorFieldLosParams {
    double max_approach_angle_h{};
    double max_approach_angle_v{};
    double proportional_gain_h{};
    double proportional_gain_v{};
    double time_step{};  // in milliseconds
};

/**
 * @brief Implements the vector field LOS guidance algorithm.
 *
 * This class computes desired heading and pitch commands using vector field
 * path-following logic based on the current vehicle position and active path
 * segment.
 */
class VectorFieldLOSGuidance {
   public:
    /**
     * @brief Constructs a VectorFieldLOSGuidance object.
     * @param params Parameters for the vector field LOS guidance algorithm.
     */
    explicit VectorFieldLOSGuidance(const VectorFieldLosParams& params);

    /**
     * @brief Destroys the VectorFieldLOSGuidance object.
     */
    ~VectorFieldLOSGuidance() = default;

    /**
     * @brief Calculates the desired LOS guidance outputs.
     * @param inputs Input values required for vector field LOS computation.
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
        const types::Inputs& inputs) const;

    /**
     * @brief Parameters used by the vector field LOS guidance algorithm.
     */
    VectorFieldLosParams m_params{};

    /**
     * @brief Stores the horizontal path angle.
     */
    double path_heading_{0.0};

    /**
     * @brief Stores the vertical path angle.
     */
    double path_pitch_{0.0};

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

#endif  // LOS_GUIDANCE__LIB__VECTOR_FIELD_LOS_HPP_

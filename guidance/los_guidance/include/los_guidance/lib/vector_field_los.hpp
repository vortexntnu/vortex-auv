#ifndef VECTOR_FIELD_LOS_GUIDANCE_HPP
#define VECTOR_FIELD_LOS_GUIDANCE_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

// Parameter Structure
struct VectorFieldLosParams {
    double max_approach_angle_h{};
    double max_approach_angle_v{};
    double k_p_h{};
    double k_p_v{};
    double time_step{};
};

// Vector Field LOS Guidance Class
class VectorFieldLOSGuidance {
   public:
    // Constructor / Destructor
    VectorFieldLOSGuidance(const VectorFieldLosParams& params);
    ~VectorFieldLOSGuidance() = default;

    // Main Output Calculation
    types::Outputs calculate_outputs(const types::Inputs& inputs);

   private:
    // Internal Update Functions
    void update_angles(const types::Inputs& inputs);
    types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs) const;

    // Parameters
    VectorFieldLosParams m_params{};

    // Path Angles
    double pi_h_{0.0};
    double pi_v_{0.0};

    // Rotation Representation
    Eigen::AngleAxisd rotation_y_{0.0, Eigen::Vector3d::UnitY()};
    Eigen::AngleAxisd rotation_z_{0.0, Eigen::Vector3d::UnitZ()};
};

}  // namespace vortex::guidance::los

#endif  // VECTOR_FIELD_LOS_GUIDANCE_HPP

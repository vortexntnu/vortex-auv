#ifndef PROPORTIONAL_LOS_GUIDANCE_HPP
#define PROPORTIONAL_LOS_GUIDANCE_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

// Parameter Structure
struct ProportionalLosParams {
    double lookahead_distance_h{};
    double lookahead_distance_v{};
};

// Proportional LOS Guidance Class
class ProportionalLOSGuidance {
   public:

    // Constructor / Destructor
    ProportionalLOSGuidance(const ProportionalLosParams& params);
    ~ProportionalLOSGuidance() = default;

    // Main Output Calculation
    types::Outputs calculate_outputs(const types::Inputs& inputs);

   private:

    // Internal Update Functions
    void update_angles(const types::Inputs& inputs);
    types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs) const;

    // Parameters
    ProportionalLosParams m_params{};

    // Path Angles
    double pi_h_{0.0};
    double pi_v_{0.0};

    // Rotation Representation
    Eigen::AngleAxisd rotation_y_{0.0, Eigen::Vector3d::UnitY()};
    Eigen::AngleAxisd rotation_z_{0.0, Eigen::Vector3d::UnitZ()};
};

}  // namespace vortex::guidance::los

#endif // PROPORTIONAL_LOS_GUIDANCE_HPP
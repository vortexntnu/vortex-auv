#ifndef PROPORTIONAL_LOS_GUIDANCE_HPP
#define PROPORTIONAL_LOS_GUIDANCE_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {
struct ProportionalLosParams {
    double lookahead_distance_h{};
    double lookahead_distance_v{};
};

class ProportionalLOSGuidance { 
   public:
    ProportionalLOSGuidance(const ProportionalLosParams& params);
    ~ProportionalLOSGuidance() = default;

    types::Outputs calculate_outputs(const types::Inputs& inputs);

   private:
    void update_angles(const types::Inputs& inputs);
    types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs) const;

    ProportionalLosParams m_params{};
    double pi_h_{0.0};
    double pi_v_{0.0};
    Eigen::AngleAxisd rotation_y_{0.0, Eigen::Vector3d::UnitY()};
    Eigen::AngleAxisd rotation_z_{0.0, Eigen::Vector3d::UnitZ()};
};

}  // namespace vortex::guidance::los

#endif  // PROPORTIONAL_LOS_GUIDANCE_HPP

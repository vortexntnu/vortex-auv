#ifndef LOS_GUIDANCE__LIB__INTEGRAL_LOS_HPP_
#define LOS_GUIDANCE__LIB__INTEGRAL_LOS_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

// Parameter Structure
struct IntegralLosParams {
    double k_p_h{};
    double k_p_v{};
    double k_i_h{};
    double k_i_v{};
    double time_step{};
};

// Integral LOS Guidance Class
class IntegralLOSGuidance {
   public:
    // Constructor / Destructor
    explicit IntegralLOSGuidance(const IntegralLosParams& params);
    ~IntegralLOSGuidance() = default;

    // Main Output Calculation
    types::Outputs calculate_outputs(const types::Inputs& inputs);

   private:
    // Internal Update Functions
    void update_angles(const types::Inputs& inputs);
    types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs);

    // Parameters
    IntegralLosParams m_params{};

    // Integral States
    double int_h{};
    double int_v{};

    // Path Angles
    double pi_h_{};
    double pi_v_{};

    // Rotation Representation
    Eigen::AngleAxisd rotation_y_{0.0, Eigen::Vector3d::UnitY()};
    Eigen::AngleAxisd rotation_z_{0.0, Eigen::Vector3d::UnitZ()};
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LIB__INTEGRAL_LOS_HPP_

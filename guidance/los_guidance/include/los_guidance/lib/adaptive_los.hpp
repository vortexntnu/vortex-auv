#ifndef LOS_GUIDANCE__LIB__ADAPTIVE_LOS_HPP_
#define LOS_GUIDANCE__LIB__ADAPTIVE_LOS_HPP_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

// Parameter Structure
struct AdaptiveLosParams {
    double lookahead_distance_h{};
    double lookahead_distance_v{};
    double gamma_h{};
    double gamma_v{};
    double time_step{};
};

// Adaptive LOS Guidance Class
class AdaptiveLOSGuidance {
   public:
    // Constructor / Destructor
    explicit AdaptiveLOSGuidance(const AdaptiveLosParams& params);
    ~AdaptiveLOSGuidance() = default;

    // Main Output Calculation
    types::Outputs calculate_outputs(const types::Inputs& inputs);

   private:
    // Internal Update Functions
    void update_angles(const types::Inputs& inputs);
    const types::CrossTrackError calculate_crosstrack_error(
        const types::Inputs& inputs);
    void update_adaptive_estimates(
        const types::CrossTrackError& cross_track_error);

    // Parameters
    AdaptiveLosParams params_{};

    // Rotation Matrices
    Eigen::Matrix3d rotation_y_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rotation_z_ = Eigen::Matrix3d::Zero();

    // Path Angles
    double pi_h_{};
    double pi_v_{};

    // Adaptive Estimates
    double beta_c_hat_ = 0.0;
    double alpha_c_hat_ = 0.0;
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LIB__ADAPTIVE_LOS_HPP_

#ifndef ADAPTIVE_LOS_GUIDANCE_HPP
#define ADAPTIVE_LOS_GUIDANCE_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "los_guidance/lib/types.hpp"
#include <cmath>
 
/**
 * @brief Adaptive Line-of-Sight (LOS) guidance algorithm based on slide 113
 * in "Fossen 2024 Lecture on 2D and 3D path-following control".
 */

namespace vortex::guidance::los {

    struct AdaptiveLosParams {
        double lookahead_distance_h{};
        double lookahead_distance_v{};
        double gamma_h{};
        double gamma_v{};
        double time_step{};
    }; 

    class AdaptiveLOSGuidance {
        public:
            AdaptiveLOSGuidance(const AdaptiveLosParams& params);
            ~AdaptiveLOSGuidance() = default;

            types::Outputs calculate_outputs(const types::Inputs& inputs);

        private:
            void update_angles(const types::Inputs& inputs);
            const types::CrossTrackError calculate_crosstrack_error(const types::Inputs& inputs);
            void update_adaptive_estimates(const types::CrossTrackError& cross_track_error);

            AdaptiveLosParams m_params{};
            Eigen::Matrix3d rotation_y_ = Eigen::Matrix3d::Zero();
            Eigen::Matrix3d rotation_z_ = Eigen::Matrix3d::Zero();
            double pi_h_{};
            double pi_v_{};
            double beta_c_hat_{};
            double alpha_c_hat_{};


    };  // namespace vortex::guidance::los

}
#endif  // LOS_GUIDANCE_HPP 
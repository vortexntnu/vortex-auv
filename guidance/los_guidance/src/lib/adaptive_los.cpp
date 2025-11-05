#include "los_guidance/lib/adaptive_los.hpp"

namespace vortex::guidance::los {
    
    AdaptiveLOSGuidance::AdaptiveLOSGuidance(const AdaptiveLosParams& params): m_params{params} {}

    void AdaptiveLOSGuidance::update_angles(const types::Inputs& inputs) {
        const double dx = inputs.next_point.x - inputs.prev_point.x;
        const double dy = inputs.next_point.y - inputs.prev_point.y;
        const double dz = inputs.next_point.z - inputs.prev_point.z;

        pi_h_ = std::atan2(dy, dx);
        pi_v_ = std::atan2(-dz, std::sqrt(dx*dx + dy*dy));

        rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
        rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
    }

    types::CrossTrackError AdaptiveLOSGuidance::calculate_crosstrack_error(const types::Inputs& inputs) const {

        const types::Point difference = inputs.current_position - inputs.prev_point;
        const Eigen::Vector3d difference_vector = difference.as_vector();

        const Eigen::Vector3d cross_track_error =
            rotation_y_.transpose() * rotation_z_.transpose() * difference_vector;

        return types::CrossTrackError::from_vector(cross_track_error);
    }

    void AdaptiveLOSGuidance::update_adaptive_estimates(const types::CrossTrackError& e) {
        const double denom_h = std::sqrt(
            m_params.lookahead_distance_h * m_params.lookahead_distance_h + e.y_e * e.y_e);
        const double denom_v = std::sqrt(
            m_params.lookahead_distance_v * m_params.lookahead_distance_v + e.z_e * e.z_e);

        const double beta_dot  = m_params.gamma_h * (m_params.lookahead_distance_h / denom_h) * e.y_e;
        const double alpha_dot = m_params.gamma_v * (m_params.lookahead_distance_v / denom_v) * e.z_e;

        beta_c_hat_  += beta_dot  * m_params.time_step;
        alpha_c_hat_ += alpha_dot * m_params.time_step;
    }
    
    types::Output AdaptiveLOSGuidance::calculate_outputs(const types::Inputs& inputs) {

        update_angles(inputs)
        const types::CrossTrackError e = calculate_crosstrack_error(inputs);
        update_adaptive_estimates(e);

        const double psi_d   = pi_h_ - beta_c_hat_ - std::atan(e.y_e / params_.lookahead_distance_h);
        const double theta_d = pi_v_ + alpha_c_hat_ + std::atan(e.z_e / params_.lookahead_distance_v);
        
        return types::Output{psi_d, theta_d};
    }

    
}  // namespace vortex::guidance

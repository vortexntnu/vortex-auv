  #include "los_guidance/lib/proportional_los.hpp"

namespace vortex::guidance::los {

    ProportionalLOSGuidance::ProportionalLOSGuidance(const ProportionalLosParams& params) : m_params{params} {}

    void ProportionalLOSGuidance::update_angles(const types::Inputs& inputs) {
        const types::Point difference = inputs.next_point - inputs.prev_point;

        pi_h_ = std::atan2(difference.y, difference.x);
        pi_v_ = std::atan2(-difference.z, std::sqrt(difference.x * difference.x + difference.y * difference.y));

        rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
        rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
    }

    types::CrossTrackError ProportionalLOSGuidance::calculate_crosstrack_error(const types::Inputs& inputs) const {

        const Eigen::Vector3d diff_vec = (inputs.current_position - inputs.prev_point).as_vector();
        const Eigen::Vector3d e_perp = rotation_y_.transpose() * rotation_z_.transpose() * diff_vec;

        return types::CrossTrackError::from_vector(e_perp);
    }

    types::Output ProportionalLOSGuidance::calculate_outputs(const types::Inputs& inputs) {
        update_angles(inputs);
        const types::CrossTrackError e = calculate_crosstrack_error(inputs);

        const double k_p_h = 1.0 / std::max(params_.lookahead_distance_h, 1e-9);
        const double k_p_v = 1.0 / std::max(params_.lookahead_distance_v, 1e-9);

        const double psi_d   = pi_h_ - std::atan(k_p_h * e.y_e);
        const double theta_d = pi_v_ + std::atan(k_p_v * e.z_e);

        return types::Output{psi_d, theta_d};
    }

}  // namespace vortex::guidance::los
#include <los_guidance/lib/adaptive_los.hpp>
#include "los_guidance/lib/types.hpp"

namespace vortex::guidance::los {

AdaptiveLOSGuidance::AdaptiveLOSGuidance(const AdaptiveLosParams& params)
    : params_{params} {} 

void AdaptiveLOSGuidance::update_angles(const types::Inputs& inputs) {
    const double dx = inputs.next_point.x - inputs.prev_point.x;
    const double dy = inputs.next_point.y - inputs.prev_point.y;
    const double dz = inputs.next_point.z - inputs.prev_point.z;

    pi_h_ = std::atan2(dy, dx);
    pi_v_ = std::atan2(-dz, std::sqrt(dx * dx + dy * dy));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
}

const types::CrossTrackError AdaptiveLOSGuidance::calculate_crosstrack_error(
    const types::Inputs& inputs) {
    const types::Point difference = inputs.current_position - inputs.prev_point;
    const Eigen::Vector3d difference_vector = difference.as_vector();

    const Eigen::Vector3d cross_track_error =
        rotation_y_.transpose() * rotation_z_.transpose() * difference_vector;

    return types::CrossTrackError::from_vector(cross_track_error);
}

void AdaptiveLOSGuidance::update_adaptive_estimates(
    const types::CrossTrackError& cross_track_error) {
    const double denom_h = std::sqrt(params_.lookahead_distance_h *
                                         params_.lookahead_distance_h +
                                     cross_track_error.y_e * cross_track_error.y_e);
    const double denom_v = std::sqrt(params_.lookahead_distance_v *
                                         params_.lookahead_distance_v +
                                     cross_track_error.z_e * cross_track_error.z_e);

    const double beta_dot =
        params_.gamma_h * (params_.lookahead_distance_h / denom_h) * cross_track_error.y_e;
    const double alpha_dot =
        params_.gamma_v * (params_.lookahead_distance_v / denom_v) * cross_track_error.z_e;

    beta_c_hat_ += beta_dot * params_.time_step;
    alpha_c_hat_ += alpha_dot * params_.time_step;
}

types::Outputs AdaptiveLOSGuidance::calculate_outputs(
    const types::Inputs& inputs) {
    update_angles(inputs);
    const types::CrossTrackError cross_track_error = calculate_crosstrack_error(inputs);
    update_adaptive_estimates(cross_track_error);

    const double psi_d =
        pi_h_ - beta_c_hat_ - std::atan(cross_track_error.y_e / params_.lookahead_distance_h);
    const double theta_d =
        pi_v_ + alpha_c_hat_ + std::atan(cross_track_error.z_e / params_.lookahead_distance_v);

    
    return types::Outputs{psi_d, theta_d};
}



}  // namespace vortex::guidance::los

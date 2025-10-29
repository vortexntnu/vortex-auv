#include "los_guidance/los_guidance.hpp"

namespace vortex::guidance {

AdaptiveLOSGuidance::AdaptiveLOSGuidance(const LOS::Params& params)
    : params_(params) {}

void AdaptiveLOSGuidance::update_angles(const LOS::Point& prev_point,
                                        const LOS::Point& next_point) {
    const LOS::Point difference = next_point - prev_point;

    pi_h_ = atan2(difference.y, difference.x);
    pi_v_ = atan2(-difference.z, sqrt(difference.x * difference.x +
                                      difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
}

LOS::CrossTrackError AdaptiveLOSGuidance::calculate_crosstrack_error(
    const LOS::Point& prev_point,
    const LOS::Point& current_position) const {
    const LOS::Point difference = current_position - prev_point;
    const Eigen::Vector3d difference_vector = difference.as_vector();

    const Eigen::Vector3d cross_track_error =
        rotation_y_.transpose() * rotation_z_.transpose() * difference_vector;

    return LOS::CrossTrackError::from_vector(cross_track_error);
}

double AdaptiveLOSGuidance::calculate_psi_d(const double& y_e) const {
    return pi_h_ - beta_c_hat_ - atan(y_e / params_.lookahead_distance_h);
}

double AdaptiveLOSGuidance::calculate_theta_d(const double& z_e) const {
    return pi_v_ + alpha_c_hat_ + atan(z_e / params_.lookahead_distance_v);
}

void AdaptiveLOSGuidance::update_adaptive_estimates(
    const LOS::CrossTrackError& crosstrack_error) {
    double beta_c_hat_dot =
        params_.gamma_h *
        (params_.lookahead_distance_h /
         sqrt(params_.lookahead_distance_h * params_.lookahead_distance_h +
              crosstrack_error.y_e * crosstrack_error.y_e)) *
        crosstrack_error.y_e;
    double alpha_c_hat_dot =
        params_.gamma_v *
        (params_.lookahead_distance_v /
         sqrt(params_.lookahead_distance_v * params_.lookahead_distance_v +
              crosstrack_error.z_e * crosstrack_error.z_e)) *
        crosstrack_error.z_e;

    beta_c_hat_ += beta_c_hat_dot * params_.time_step;
    alpha_c_hat_ += alpha_c_hat_dot * params_.time_step;
}

// ---------------- Proportional LOS Guidance Implementation ----------------

ProportionalLOSGuidance::ProportionalLOSGuidance(const LOS::Params& params)
    : params_(params) {}

void ProportionalLOSGuidance::update_angles(const LOS::Point& prev_point,
                                            const LOS::Point& next_point) {
    const LOS::Point difference = next_point - prev_point;

    pi_h_ = std::atan2(difference.y, difference.x);
    pi_v_ = std::atan2(-difference.z,
                       std::sqrt(difference.x * difference.x +
                                 difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
}

LOS::CrossTrackError ProportionalLOSGuidance::calculate_crosstrack_error(
    const LOS::Point& prev_point,
    const LOS::Point& current_position) const {
    const Eigen::Vector3d diff_vec = (current_position - prev_point).as_vector();

    const Eigen::Vector3d e_perp =
        rotation_y_.transpose() * rotation_z_.transpose() * diff_vec;

    return LOS::CrossTrackError::from_vector(e_perp);
}

double ProportionalLOSGuidance::calculate_psi_d(const double& y_e) const {
    const double k_p_h = 1.0 / std::max(params_.lookahead_distance_h, 1e-9);
    return pi_h_ - std::atan(k_p_h * y_e);
}

double ProportionalLOSGuidance::calculate_theta_d(const double& z_e) const {
    const double k_p_v = 1.0 / std::max(params_.lookahead_distance_v, 1e-9);
    return pi_v_ + std::atan(k_p_v * z_e);
}

}  // namespace vortex::guidance

#include <los_guidance/los_guidance.hpp>

void AdaptiveLOSGuidance::update_angles(const LOS::Point& prev_point, const LOS::Point& next_point) {
    const LOS::Point difference = next_point - prev_point;

    pi_h_ = atan2(difference.y, difference.x);
    pi_v_ = atan2(-difference.z, sqrt(difference.x * difference.x + difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
}

LOS::CrossTrackError AdaptiveLOSGuidance::calculate_crosstrack_error(const LOS::Point& prev_point, const LOS::Point& current_position) {
    const LOS::Point difference = current_position - prev_point;
    const Eigen::Vector3d difference_vector = difference.as_vector();

    const Eigen::Vector3d cross_track_error = rotation_y_.transpose() * rotation_z_.transpose() * difference_vector;

    return LOS::CrossTrackError::from_vector(cross_track_error);
}

double AdaptiveLOSGuidance::calculate_psi_d(const double& y_e) {
    return pi_h_ - beta_c_hat_ - atan(y_e / lookahead_distance_h_);
}

double AdaptiveLOSGuidance::calculate_theta_d(const double& z_e) {
    return pi_v_ + alpha_c_hat_ + atan(z_e / lookahead_distance_v_);
}

void AdaptiveLOSGuidance::update_adaptive_estimates(const LOS::CrossTrackError& crosstrack_error) {
    double beta_c_hat_dot = gamma_h_ * (lookahead_distance_h_ / sqrt(lookahead_distance_h_ * lookahead_distance_h_ + crosstrack_error.y_e * crosstrack_error.y_e)) * crosstrack_error.y_e;
    double alpha_c_hat_dot = gamma_v_ * (lookahead_distance_v_ / sqrt(lookahead_distance_v_ * lookahead_distance_v_ + crosstrack_error.z_e * crosstrack_error.z_e)) * crosstrack_error.z_e;

    beta_c_hat_ += beta_c_hat_dot * time_step_;
    alpha_c_hat_ += alpha_c_hat_dot * time_step_;
}
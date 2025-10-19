#include <guidance_velocity_controller/guidance.hpp>

LOSGuidance::LOSGuidance(
    double lookahead_distance_h_,
    double lookahead_distance_v_,
    double gamma_h_,
    double gamma_v_,
    double time_step_,
    double u_desired_,
    double u_min_,
    double d_scale_)
    : lookahead_distance_h(lookahead_distance_h_),
      lookahead_distance_v(lookahead_distance_v_),
      gamma_h(gamma_h_),
      gamma_v(gamma_v_),
      time_step(time_step_),
      u_desired(u_desired_),
      u_min(u_min_),
      d_scale(d_scale_) {
    rotation_y_ = Eigen::Matrix3d::Identity();
    rotation_z_ = Eigen::Matrix3d::Identity();
};

void LOSGuidance::update_angles(const LOS::Point& new_point, const LOS::Point& prev_point) {
    const LOS::Point difference = new_point - prev_point;

    pi_h_ = atan2(difference.y, difference.x);
    pi_v_ = atan2(-difference.z, sqrt(difference.x * difference.x + difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY()).toRotationMatrix();
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ()).toRotationMatrix();

};

void LOSGuidance::cross_track_error(
    const LOS::Point& current_position, const LOS::Point& prev_point) {
    const LOS::Point difference = current_position - prev_point;
    const Eigen::Vector3d difference_vector = difference.as_vector();

    const Eigen::Vector3d cross_track_error = rotation_y_.transpose() * rotation_z_.transpose() * difference_vector;

    //save the cross track error
    cte = LOS::CrossTrackError::from_vector(cross_track_error);
}

LOS::CrossTrackError& LOSGuidance::cross_track() {
    return cte; 
};

void LOSGuidance::update_adaptive_estimates() {
    double beta_c_hat_dot =
        gamma_h *
        (lookahead_distance_h /
         sqrt(lookahead_distance_h* lookahead_distance_h +
              cte.y_e * cte.y_e)) *
        cte.y_e;
    double alpha_c_hat_dot =
        gamma_v *
        (lookahead_distance_v /
         sqrt(lookahead_distance_v* lookahead_distance_v +
              cte.z_e * cte.z_e))*
        cte.z_e;

    beta_c_hat_ += beta_c_hat_dot * time_step;
    alpha_c_hat_ += alpha_c_hat_dot * time_step;
}

double LOSGuidance::desired_surge_speed(const LOS::Point& destination , const LOS::Point& current_position) {
    double dx = destination.x - current_position.x;
    double dy = destination.y - current_position.y;
    double dz = destination.z - current_position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    double d = u_min + (u_desired - u_min) * std::tanh(distance/d_scale);

    return d;
}

double LOSGuidance::desired_heading() {
    return pi_h_ - beta_c_hat_ - atan(cte.y_e / lookahead_distance_h);
}

double LOSGuidance::desired_pitch() {
    return pi_v_ + alpha_c_hat_ + atan(cte.z_e / lookahead_distance_v);
}

Eigen::Vector3d LOSGuidance::get_outputs() const {
    return Eigen::Vector3d::Zero();
}

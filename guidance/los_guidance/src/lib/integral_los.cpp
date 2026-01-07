#include <los_guidance/lib/integral_los.hpp>

namespace vortex::guidance::los {

IntegralLOSGuidance::IntegralLOSGuidance(const IntegralLosParams& params)
    : m_params{params} {}

void IntegralLOSGuidance::update_angles(const types::Inputs& inputs) {
    const types::Point difference = inputs.next_point - inputs.prev_point;

    pi_h_ = std::atan2(difference.y, difference.x);
    pi_v_ = std::atan2(-difference.z, std::sqrt(difference.x * difference.x +
                                                difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ()); 
}

types::CrossTrackError IntegralLOSGuidance::calculate_crosstrack_error(
    const types::Inputs& inputs) {
    const Eigen::Vector3d diff_vec =
        (inputs.current_position - inputs.prev_point).as_vector();
    const Eigen::Vector3d path_frame_error = rotation_y_.toRotationMatrix().transpose() *
                                   rotation_z_.toRotationMatrix().transpose() *
                                   diff_vec;

    return types::CrossTrackError::from_vector(path_frame_error);
}

types::Outputs IntegralLOSGuidance::calculate_outputs(
    const types::Inputs& inputs) {
    update_angles(inputs);
    const types::CrossTrackError cross_track_error = calculate_crosstrack_error(inputs);

    int_h += cross_track_error.y_e * m_params.time_step;
    int_v += cross_track_error.z_e * m_params.time_step;

    const double u_h = m_params.k_p_h * cross_track_error.y_e + m_params.k_i_h * int_h;
    const double u_v = m_params.k_p_v * cross_track_error.z_e + m_params.k_i_v * int_v;

    const double psi_d = pi_h_ - std::atan(u_h);
    const double theta_d = pi_v_ + std::atan(u_v);

    return types::Outputs{psi_d, theta_d};
}

}  // namespace vortex::guidance::los

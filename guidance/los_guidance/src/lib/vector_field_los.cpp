#include <los_guidance/lib/vector_field_los.hpp>

namespace vortex::guidance::los {

VectorFieldLOSGuidance::VectorFieldLOSGuidance(const VectorFieldLosParams& params)
    : m_params{params} {}

void VectorFieldLOSGuidance::update_angles(const types::Inputs& inputs) {
    const types::Point difference = inputs.next_point - inputs.prev_point;

    pi_h_ = std::atan2(difference.y, difference.x);
    pi_v_ = std::atan2(-difference.z, std::sqrt(difference.x * difference.x +
                                                difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(pi_v_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(pi_h_, Eigen::Vector3d::UnitZ());
}

types::CrossTrackError VectorFieldLOSGuidance::calculate_crosstrack_error(
    const types::Inputs& inputs) const {
    const Eigen::Vector3d diff_vec =
        (inputs.current_position - inputs.prev_point).as_vector();

    const Eigen::Vector3d path_frame_error = rotation_y_.toRotationMatrix().transpose() *
                                   rotation_z_.toRotationMatrix().transpose() *
                                   diff_vec;

    return types::CrossTrackError::from_vector(path_frame_error);
}

types::Outputs VectorFieldLOSGuidance::calculate_outputs(
    const types::Inputs& inputs) {
    update_angles(inputs);
    const types::CrossTrackError cross_track_error = calculate_crosstrack_error(inputs);

    const double approach_h =
        m_params.max_approach_angle_h * (2.0 / M_PI) * std::atan(m_params.k_p_h * cross_track_error.y_e);

    const double approach_v =
        m_params.max_approach_angle_v * (2.0 / M_PI) * std::atan(m_params.k_p_v * cross_track_error.z_e);

    const double psi_d = pi_h_ - approach_h;
    
    const double theta_d = pi_v_ + approach_v;

    return types::Outputs{psi_d, theta_d};
}

}  // namespace vortex::guidance::los

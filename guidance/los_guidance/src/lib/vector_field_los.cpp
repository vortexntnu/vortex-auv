#include <los_guidance/lib/vector_field_los.hpp>

namespace vortex::guidance::los {

// Constructor
VectorFieldLOSGuidance::VectorFieldLOSGuidance(
    const VectorFieldLosParams& params)
    : m_params{params} {}
 
// Angle Update
void VectorFieldLOSGuidance::update_angles(const types::Inputs& inputs) {
    const types::Point difference = inputs.next_point - inputs.prev_point;

    path_heading_ = std::atan2(difference.y, difference.x);
    path_pitch_ = std::atan2(-difference.z, std::sqrt(difference.x * difference.x +
                                                difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(path_pitch_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(path_heading_, Eigen::Vector3d::UnitZ());
}

// Cross-Track Error Calculation
types::CrossTrackError VectorFieldLOSGuidance::calculate_crosstrack_error(
    const types::Inputs& inputs) const {
    const Eigen::Vector3d diff_vec =
        (inputs.current_position - inputs.prev_point).as_vector();

    const Eigen::Vector3d path_frame_error =
        rotation_y_.toRotationMatrix().transpose() *
        rotation_z_.toRotationMatrix().transpose() * diff_vec;

    return types::CrossTrackError::from_vector(path_frame_error);
}

// Output Calculation
types::Outputs VectorFieldLOSGuidance::calculate_outputs(
    const types::Inputs& inputs) {
    update_angles(inputs);

    const types::CrossTrackError cross_track_error =
        calculate_crosstrack_error(inputs);

    const double approach_h = m_params.max_approach_angle_h * (2.0 / M_PI) *
                              std::atan(m_params.proportional_gain_h * cross_track_error.y_e);

    const double approach_v = m_params.max_approach_angle_v * (2.0 / M_PI) *
                              std::atan(m_params.proportional_gain_v * cross_track_error.z_e);

    const double desired_yaw = path_heading_ - approach_h;
    const double desired_pitch = path_pitch_ - approach_v;

    return types::Outputs{desired_yaw, desired_pitch};
}

}  // namespace vortex::guidance::los

#include <los_guidance/lib/integral_los.hpp>

namespace vortex::guidance::los {

// Constructor
IntegralLOSGuidance::IntegralLOSGuidance(const IntegralLosParams& params)
    : m_params(params) {
    if (params.proportional_gain_h <= 0.0 ||
        params.proportional_gain_v <= 0.0 || params.integral_gain_h <= 0.0 ||
        params.integral_gain_v <= 0.0 || params.time_step <= 0.0) {
        throw std::invalid_argument(
            "IntegralLOSGuidance: all params must be > 0");
    }
}

// Angle Update
void IntegralLOSGuidance::update_angles(const types::Inputs& inputs) {
    const types::Point difference = inputs.next_point - inputs.prev_point;

    path_heading_ = std::atan2(difference.y, difference.x);
    path_pitch_ = std::atan2(
        -difference.z,
        std::sqrt(difference.x * difference.x + difference.y * difference.y));

    rotation_y_ = Eigen::AngleAxisd(path_pitch_, Eigen::Vector3d::UnitY());
    rotation_z_ = Eigen::AngleAxisd(path_heading_, Eigen::Vector3d::UnitZ());
}

// Cross-Track Error Calculation
types::CrossTrackError IntegralLOSGuidance::calculate_crosstrack_error(
    const types::Inputs& inputs) {
    const Eigen::Vector3d diff_vec =
        (inputs.current_position - inputs.prev_point).as_vector();

    const Eigen::Vector3d path_frame_error =
        rotation_y_.toRotationMatrix().transpose() *
        rotation_z_.toRotationMatrix().transpose() * diff_vec;

    return types::CrossTrackError::from_vector(path_frame_error);
}

// Output Calculation
types::Outputs IntegralLOSGuidance::calculate_outputs(
    const types::Inputs& inputs) {
    update_angles(inputs);

    const types::CrossTrackError cross_track_error =
        calculate_crosstrack_error(inputs);

    integrated_horizontal_error_ += cross_track_error.y_e * m_params.time_step;
    integrated_vertical_error_ += cross_track_error.z_e * m_params.time_step;

    const double u_h = m_params.proportional_gain_h * cross_track_error.y_e +
                       m_params.integral_gain_h * integrated_horizontal_error_;
    const double u_v = m_params.proportional_gain_v * cross_track_error.z_e +
                       m_params.integral_gain_v * integrated_vertical_error_;

    const double desired_yaw = path_heading_ - std::atan(u_h);
    const double desired_pitch = path_pitch_ + std::atan(u_v);

    return types::Outputs{desired_yaw, desired_pitch};
}

}  // namespace vortex::guidance::los

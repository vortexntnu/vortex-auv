#include "pid_controller_dp/pid_controller_utils.hpp"
#include <algorithm>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/typedefs.hpp"

types::Matrix3d calculate_R_quat(const types::Eta& eta) {
    return eta.as_rotation_matrix();
}

types::Matrix4x3d calculate_T_quat(const types::Eta& eta) {
    return eta.as_transformation_matrix();
}

types::Matrix6x7d calculate_J_sudo_inv(const types::Eta& eta) {
    auto J_matrix = eta.as_j_matrix();
    Eigen::MatrixXd J_pseudo_inv_dynamic =
        vortex::utils::math::pseudo_inverse(J_matrix);

    types::Matrix6x7d J_pseudo_inv;
    J_pseudo_inv = J_pseudo_inv_dynamic;
    return J_pseudo_inv;
}

types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d) {
    return eta - eta_d;
}

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val) {
    return vortex::utils::math::clamp_values(values, min_val, max_val);
}

types::Vector7d anti_windup(const double dt,
                            const types::Eta& error,
                            const types::Vector7d& integral) {
    return vortex::utils::math::anti_windup(dt, error.to_vector(), integral,
                                            -80.0, 80.0);
}

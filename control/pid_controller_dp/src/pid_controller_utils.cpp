#include "pid_controller_dp/pid_controller_utils.hpp"
#include <algorithm>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/typedefs.hpp"

// void print_J_transformation(const types::J_transformation& J) {
//     spdlog::info("J_transformation:");

//     spdlog::info("R (3x3) elements:");
//     for (int i = 0; i < J.R.rows(); ++i) {
//         for (int j = 0; j < J.R.cols(); ++j) {
//             spdlog::info("R[{},{}] = {}", i, j, J.R(i, j));
//         }
//     }

//     spdlog::info("T (4x3) elements:");
//     for (int i = 0; i < J.T.rows(); ++i) {
//         for (int j = 0; j < J.T.cols(); ++j) {
//             spdlog::info("T[{},{}] = {}", i, j, J.T(i, j));
//         }
//     }

//     spdlog::info("Combined Matrix (7x6) elements:");
//     auto M = J.as_matrix();
//     for (int i = 0; i < M.rows(); ++i) {
//         for (int j = 0; j < M.cols(); ++j) {
//             spdlog::info("M[{},{}] = {}", i, j, M(i, j));
//         }
//     }
// }

// void print_Jinv_transformation(const types::Matrix6x7d& J_inv) {
//     spdlog::info("J (6x7) elements:");
//     for (int i = 0; i < J_inv.rows(); ++i) {
//         for (int j = 0; j < J_inv.cols(); ++j) {
//             spdlog::info("J_inv[{},{}] = {}", i, j, J_inv(i, j));
//         }
//     }
// }

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
    // return values.cwiseMax(min_val).cwiseMin(max_val);
    return vortex::utils::math::clamp_values(values, min_val, max_val);
}

types::Vector7d anti_windup(const double dt,
                            const types::Eta& error,
                            const types::Vector7d& integral) {
    // Eigen::VectorXd integral_eig = integral;
    // Eigen::VectorXd updated = vortex::utils::math::anti_windup(
    //     dt, error.to_vector(), integral_eig, -80.0, 80.0);

    // types::Vector7d integral_anti_windup;
    // for (int i = 0; i < 7; ++i) {
    //     integral_anti_windup(i) = updated(i);
    // }

    // return integral_anti_windup;
    return vortex::utils::math::anti_windup(dt, error.to_vector(), integral,
                                            -80.0, 80.0);
}

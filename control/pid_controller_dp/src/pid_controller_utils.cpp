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
    return eta.ori.normalized().toRotationMatrix();
}

types::Matrix4x3d calculate_T_quat(const types::Eta& eta) {
    types::Quaterniond quaternion_norm = eta.ori.normalized();

    double w = quaternion_norm.w();
    double x = quaternion_norm.x();
    double y = quaternion_norm.y();
    double z = quaternion_norm.z();

    types::Matrix4x3d transformation_matrix;

    transformation_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    // transformation_matrix << -x, -y, -z, w, -z, y, -z, w, -x, -y, x, w;

    return transformation_matrix * 0.5;
}

types::Matrix6x7d calculate_J_sudo_inv(const types::Eta& eta) {
    types::Eta eta_norm;

    eta_norm.pos = eta.pos;
    eta_norm.ori = eta.ori;

    types::Matrix3d R = calculate_R_quat(eta_norm);
    types::Matrix4x3d T = calculate_T_quat(eta_norm);

    types::J_transformation J;
    J.R = R;
    J.T = T;
    // print_J_transformation(J);

    types::Matrix6x7d J_transpose = J.as_matrix().transpose();
    // spdlog::info("J_transpose (6x7) elements:");
    // print_Jinv_transformation(J_transpose);
    // types::Matrix6x7d J_inv = (J.as_matrix() * J_transpose).inverse();

    // spdlog::info("");
    // print_Jinv_transformation(J_inv);

    // (J_transpose * J.as_matrix()).inverse() * J_transpose

    // types::Matrix6x7d J_pseudo_inv =
    //     (J_transpose * J.as_matrix()).inverse() * J_transpose;

    types::Matrix6x7d J_pseudo_inv =
        vortex::utils::math::pseudo_inverse(J.as_matrix());

    return J_pseudo_inv;
}

types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d) {
    types::Eta eta_error;
    // vortex::utils::types::EtaQuat eta_error;

    eta_error.pos = eta.pos - eta_d.pos;
    eta_error.ori = eta_d.ori.conjugate() * eta.ori;

    eta_error.ori = eta_error.ori.normalized();

    return eta_error;
}

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val) {
    return values.cwiseMax(min_val).cwiseMin(max_val);
    // return vortex::utils::math::clamp_values(values, min_val, max_val);
}

types::Vector7d anti_windup(const double dt,
                            const types::Eta& error,
                            const types::Vector7d& integral) {
    types::Eta error_norm;

    error_norm.pos = error.pos;
    error_norm.ori = error.ori;

    types::Vector7d integral_anti_windup =
        integral + (error_norm.as_vector() * dt);

    integral_anti_windup = clamp_values(integral_anti_windup, -80.0, 80.0);
    return integral_anti_windup;
    // return vortex::utils::math::anti_windup(dt, error_norm.as_vector(),
    //                                         integral, -80.0, 80.0);
}

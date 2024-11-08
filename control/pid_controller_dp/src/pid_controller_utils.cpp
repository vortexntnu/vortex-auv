#include "pid_controller_dp/pid_controller_utils.hpp"
#include <iostream>

Eigen::Matrix6d float64multiarray_to_diagonal_matrix6d(
    const std_msgs::msg::Float64MultiArray& msg) {
    Eigen::Matrix6d matrix = Eigen::Matrix6d::Zero();

    if (msg.data.size() != 6) {
        throw std::runtime_error(
            "Float64MultiArray message must have exactly 6 elements.");
    }

    for (size_t i = 0; i < 6; ++i) {
        matrix(i, i) = msg.data[i];
    }

    return matrix;
}

Eigen::Matrix3d calculate_R_quat(const Eigen::Vector4d& q) {
    Eigen::Vector4d q_norm = q / q.norm();

    double nu = q_norm(0);
    double eps_1 = q_norm(1);
    double eps_2 = q_norm(2);
    double eps_3 = q_norm(3);

    double r11 = 1 - (2 * ((eps_2 * eps_2) + (eps_3 * eps_3)));
    double r12 = 2 * ((eps_1 * eps_2) + (eps_3 * nu));
    double r13 = 2 * ((eps_1 * eps_3) - (eps_2 * nu));
    double r21 = 2 * ((eps_1 * eps_2) - (eps_3 * nu));
    double r22 = 1 - (2 * ((eps_1 * eps_1) + (eps_3 * eps_3)));
    double r23 = 2 * ((eps_2 * eps_3) + (eps_1 * nu));
    double r31 = 2 * ((eps_1 * eps_3) + (eps_2 * nu));
    double r32 = 2 * ((eps_2 * eps_3) - (eps_1 * nu));
    double r33 = 1 - (2 * ((eps_1 * eps_1) + (eps_2 * eps_2)));

    Eigen::Matrix3d R;
    R << r11, r21, r31, r12, r22, r32, r13, r23, r33;

    return R;
}

Eigen::Matrix4x3d calculate_T_quat(const Eigen::Vector4d& q) {
    Eigen::Vector4d q_norm = q / q.norm();

    double w = q_norm(0);
    double x = q_norm(1);
    double y = q_norm(2);
    double z = q_norm(3);

    Eigen::Matrix4x3d transformation_matrix;

    transformation_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;

    transformation_matrix = transformation_matrix * 0.5;

    return transformation_matrix;
}

Eigen::Matrix7x6d calculate_J(const Eigen::Vector7d& eta) {
    Eigen::Vector4d q = eta.tail<4>();
    q = q / q.norm();

    Eigen::Matrix3d R = calculate_R_quat(q);
    Eigen::Matrix4x3d T = calculate_T_quat(q);

    Eigen::Matrix7x6d J = Eigen::Matrix7x6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<4, 3>(3, 3) = T;

    return J;
}

Eigen::Matrix6x7d calculate_J_sudo_inv(const Eigen::Vector7d& eta) {
    Eigen::Vector7d eta_nom = eta;
    Eigen::Vector4d q = eta_nom.tail<4>();
    q = q / q.norm();
    eta_nom.tail<4>() = q;

    Eigen::Matrix7x6d J = calculate_J(eta_nom);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double tolerance = 1e-6;
    Eigen::VectorXd singularValuesInv = svd.singularValues().unaryExpr(
        [&](double x) { return (std::abs(x) > tolerance) ? 1.0 / x : 0.0; });

    Eigen::MatrixXd J_pseudo_inv = svd.matrixV() *
                                   singularValuesInv.asDiagonal() *
                                   svd.matrixU().transpose();

    return J_pseudo_inv;
}

Eigen::Vector7d error_eta(const Eigen::Vector7d& eta,
                          const Eigen::Vector7d& eta_d) {
    Eigen::Vector7d error = Eigen::Vector7d::Zero();

    error.head<3>() = eta.head<3>() - eta_d.head<3>();

    Eigen::Vector4d q_eta = eta.tail<4>();
    Eigen::Vector4d q_eta_d(eta_d(3), -eta_d(4), -eta_d(5), -eta_d(6));

    q_eta = q_eta / q_eta.norm();
    q_eta_d = q_eta_d / q_eta_d.norm();

    Eigen::Vector4d q_error(
        (q_eta_d(0) * q_eta(0)) -
            (q_eta_d(1) * q_eta(1) + q_eta_d(2) * q_eta(2) +
             q_eta_d(3) * q_eta(3)),
        (q_eta_d(0) * q_eta(1)) + (q_eta(0) * q_eta_d(1)) +
            ((-q_eta_d(3) * q_eta(2)) + (q_eta_d(2) * q_eta(3))),
        (q_eta_d(0) * q_eta(2)) + (q_eta(0) * q_eta_d(2)) +
            ((q_eta_d(3) * q_eta(1)) + (-q_eta_d(1) * q_eta(3))),
        (q_eta_d(0) * q_eta(3)) + (q_eta(0) * q_eta_d(3)) +
            ((-q_eta_d(2) * q_eta(1)) + (q_eta_d(1) * q_eta(2))));

    q_error = q_error / q_error.norm();

    error(3) = q_error(0);
    error(4) = q_error(1);
    error(5) = q_error(2);
    error(6) = q_error(3);

    return error;
}

Eigen::Vector7d anti_windup(const double dt,
                            const Eigen::Vector7d& error,
                            const Eigen::Vector7d& integral) {
    Eigen::Vector7d error_nom = error;
    Eigen::Vector4d q_error = error.tail<4>();
    q_error = q_error / q_error.norm();
    error_nom.tail<4>() = q_error;

    Eigen::Vector7d integral_anti_windup = integral + (error_nom * dt);

    for (int i = 0; i < integral_anti_windup.size(); ++i) {
        if (integral_anti_windup[i] > 30.0) {
            integral_anti_windup[i] = 30.0;
        } else if (integral_anti_windup[i] < -30.0) {
            integral_anti_windup[i] = -30.0;
        }
    }

    return integral_anti_windup;
}

Eigen::Vector6d limit_input(const Eigen::Vector6d& input) {
    Eigen::Vector6d limited_input = input;

    for (int i = 0; i < limited_input.size(); ++i) {
        if (limited_input[i] > 95.0) {
            limited_input[i] = 95.0;
        } else if (limited_input[i] < -95.0) {
            limited_input[i] = -95.0;
        }
    }

    return limited_input;
}

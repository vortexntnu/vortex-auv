#include "pid_controller_dp/pid_controller_utils.hpp"

Eigen::Matrix6d float64multiarray_to_diagonal_matrix6d(const std_msgs::msg::Float64MultiArray &msg) {
    Eigen::Matrix6d matrix = Eigen::Matrix6d::Zero();

    if (msg.data.size() != 6) {
        throw std::runtime_error("Float64MultiArray message must have exactly 6 elements.");
    }

    for (size_t i = 0; i < 6; ++i) {
        matrix(i, i) = msg.data[i]; 
    }

    return matrix;
}

double ssa(double angle) {
    double angle_ssa = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    return angle_ssa;
}

Eigen::Vector6d apply_ssa(const Eigen::Vector6d &eta) {
    Eigen::Vector6d eta_ssa = eta;

    eta_ssa.tail<3>() = eta_ssa.tail<3>().unaryExpr(&ssa);

    return eta_ssa;
}

Eigen::Matrix3d calculate_R(const Eigen::Vector6d &eta) {
    double phi = eta(3);
    double theta = eta(4);
    double psi = eta(5);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);
    double cpsi = cos(psi);
    double spsi = sin(psi);

    Eigen::Matrix3d rotation_matrix;

    rotation_matrix << cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi,
                       spsi * ctheta, spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi,
                       -stheta, ctheta * sphi, ctheta * cphi;

    return rotation_matrix;
}

Eigen::Matrix3d calculate_T(const Eigen::Vector6d &eta) {
    double phi = eta(3);
    double theta = eta(4);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);

    if (ctheta == 0) {
        throw std::runtime_error("Division by zero in calculate_T.");
    }

    Eigen::Matrix3d transformation_matrix;

    transformation_matrix << 1, sphi * stheta / ctheta, cphi * stheta / ctheta,
                             0, cphi, -sphi,
                             0, sphi / ctheta, cphi / ctheta;

    return transformation_matrix;
}

Eigen::Matrix6d calculate_J(const Eigen::Vector6d &eta) {
    Eigen::Matrix3d R = calculate_R(eta);
    Eigen::Matrix3d T = calculate_T(eta);

    Eigen::Matrix6d J = Eigen::Matrix6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<3, 3>(3, 3) = T;

    return J;
}


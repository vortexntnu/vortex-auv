#include "pid_controller_dp/pid_controller_utils.hpp"
#include <iostream>

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

Eigen::Vector7d error_eta(const Eigen::Vector7d &eta, const Eigen::Vector7d &eta_d) {
    
    Eigen::Vector7d error = Eigen::Vector7d::Zero();

    error.head<3>() = eta.head<3>() - eta_d.head<3>();

    tf2::Quaternion q_eta(eta(4), eta(5), eta(6), eta(3));
    tf2::Quaternion q_eta_d(eta_d(4), eta_d(5), eta_d(6), eta_d(3));

    tf2::Quaternion q_error = q_eta_d.inverse() * q_eta;
    q_error.normalize();

    error(3) = q_error.w();
    error(4) = q_error.x();
    error(5) = q_error.y();
    error(6) = q_error.z();

    return error;
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

Eigen::Matrix4x3d Macalculate_T_quat(const Eigen::Vector7d &eta) {

    double w = eta(3);
    double x = eta(4);
    double y = eta(5);
    double z = eta(6);

    Eigen::Matrix4x3d transformation_matrix;

    transformation_matrix << -x , -y , -z,
                             w , -z , y,
                             z , w , -x,
                             -y , x , w;

    transformation_matrix *= 0.5; 

    return transformation_matrix;
}

Eigen::Matrix7x6d calculate_J(const Eigen::Vector7d &eta) {
    Eigen::Matrix3d R = calculate_R(eta);
    Eigen::Matrix4x3d T = calculate_T(eta);

    Eigen::Matrix7x6d J = Eigen::Matrix7x6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<4, 3>(3, 3) = T;

    return J;
}

Eigen::Matrix6x7d calculate_J_sudo_inv(const Eigen::Vector7d &eta) {
    Eigen::Matrix7x6d J = calculate_J(eta);
    Eigen::Matrix6x7d J_sudo_inv = J.transpose() * (J * J.transpose()).inverse();

    return J_sudo_inv;
}

Eigen::Vector6d anti_windup(const double dt, const Eigen::Vector6d &error, const Eigen::Vector6d &integral) {
    Eigen::Vector6d integral_anti_windup = integral + (error * dt);

    for (int i = 0; i < integral_anti_windup.size(); ++i) {
        if (integral_anti_windup[i] > 30.0) {
            integral_anti_windup[i] = 30.0;
        } else if (integral_anti_windup[i] < -30.0) {
            integral_anti_windup[i] = -30.0;
        }
    }

    return integral_anti_windup;
}


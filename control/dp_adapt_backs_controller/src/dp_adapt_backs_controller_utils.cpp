#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include <algorithm>
#include <iostream>
#include "dp_adapt_backs_controller/typedefs.hpp"

double ssa(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

dp_types::Eta error_eta(const dp_types::Eta& eta, const dp_types::Eta& eta_d) {
    dp_types::Eta eta_error;
    eta_error.pos = eta.pos - eta_d.pos;
    eta_error.ori.x() = ssa(eta.ori.x() - eta_d.ori.x());
    eta_error.ori.y() = ssa(eta.ori.y() - eta_d.ori.y());
    eta_error.ori.z() = ssa(eta.ori.z() - eta_d.ori.z());
    return eta_error;
}

dp_types::Matrix3d skew_symmetric(const dp_types::Vector3d& vec) {
    dp_types::Matrix3d skew;

    skew << 0, -vec.z(), vec.y(), vec.z(), 0, -vec.x(), -vec.y(), vec.x(), 0;

    return skew;
}

dp_types::Vector3d quat_to_euler(double w, double x, double y, double z) {
    dp_types::Vector3d euler;
    euler.x() = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    euler.y() = std::asin(2 * (w * y - z * x));
    euler.z() = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

    return euler;
}

dp_types::Matrix3d calculate_R(const dp_types::Eta& eta) {
    const double roll = eta.ori.x();
    const double pitch = eta.ori.y();
    const double yaw = eta.ori.z();

    const Eigen::AngleAxisd roll_angle(roll, dp_types::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, dp_types::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, dp_types::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

    return q.toRotationMatrix();
}

dp_types::Matrix3d calculate_T(const dp_types::Eta& eta) {
    double sin_phi = std::sin(eta.ori.x());
    double cos_phi = std::cos(eta.ori.x());
    double cos_theta = std::cos(eta.ori.y());
    double tan_theta = std::tan(eta.ori.y());

    dp_types::Matrix3d T;
    T(0, 0) = 1;
    T(0, 1) = sin_phi * tan_theta;
    T(0, 2) = cos_phi * tan_theta;
    T(1, 0) = 0;
    T(1, 1) = cos_phi;
    T(1, 2) = -sin_phi;
    T(2, 0) = 0;
    T(2, 1) = sin_phi / cos_theta;
    T(2, 2) = cos_phi / cos_theta;

    return T;
}

dp_types::Matrix6d calculate_J(const dp_types::Eta& eta) {
    dp_types::Matrix3d R = calculate_R(eta);
    dp_types::Matrix3d T = calculate_T(eta);

    dp_types::J_matrix J;
    J.R = R;
    J.T = T;

    return J.as_matrix();
}

dp_types::Matrix6d calculate_J_inv(const dp_types::Eta& eta) {
    dp_types::Matrix6d J = calculate_J(eta);
    dp_types::Matrix6d J_inv = dp_types::Matrix6d::Zero();

    constexpr double tolerance = 1e-8;

    if (std::abs(J.determinant()) < tolerance) {
        std::cerr << "J(eta) is singular" << std::endl;

        // Moore-Penrose pseudoinverse in case of near singular matrix, better
        // result for smaller singular values
        J_inv = J.completeOrthogonalDecomposition().pseudoInverse();
        return J_inv;
    }

    J_inv = J.inverse();

    return J_inv;
}

dp_types::Matrix3d calculate_R_dot(const dp_types::Eta& eta,
                                   const dp_types::Nu& nu) {
    dp_types::Matrix3d R = calculate_R(eta);

    dp_types::Matrix3d S = skew_symmetric(nu.angular_speed);

    dp_types::Matrix3d R_dot = R * S;

    return R_dot;
}

dp_types::Matrix3d calculate_T_dot(const dp_types::Eta& eta,
                                   const dp_types::Nu& nu,
                                   const dp_types::Matrix6d& J) {
    double cos_phi = std::cos(eta.ori.x());
    double sin_phi = std::sin(eta.ori.x());
    double cos_theta = std::cos(eta.ori.y());
    double sin_theta = std::sin(eta.ori.y());
    double tan_theta = sin_theta / cos_theta;
    double inv_cos2 = 1 / (cos_theta * cos_theta);

    dp_types::Vector6d eta_dot = J * nu.as_vector();

    double phi_dot = eta_dot(3);
    double theta_dot = eta_dot(4);

    dp_types::Matrix3d dt_dphi;
    dt_dphi << 0.0, cos_phi * tan_theta * phi_dot,
        -sin_phi * tan_theta * phi_dot, 0.0, -sin_phi * phi_dot,
        -cos_phi * phi_dot, 0.0, (cos_phi * phi_dot) / cos_theta,
        (-sin_phi * phi_dot) / cos_theta;

    dp_types::Matrix3d dt_dtheta;
    dt_dtheta << 0.0, sin_phi * inv_cos2 * theta_dot,
        cos_phi * inv_cos2 * theta_dot, 0.0, 0.0, 0.0, 0.0,
        (sin_phi * sin_theta) * inv_cos2 * theta_dot,
        (cos_phi * sin_theta) * inv_cos2 * theta_dot;

    dp_types::Matrix3d T_dot = dt_dphi + dt_dtheta;

    return T_dot;
}
dp_types::Matrix6d calculate_J_dot(const dp_types::Eta& eta,
                                   const dp_types::Nu& nu) {
    dp_types::Matrix6d J = calculate_J(eta);
    dp_types::Matrix3d R_dot = calculate_R_dot(eta, nu);
    dp_types::Matrix3d T_dot = calculate_T_dot(eta, nu, J);

    dp_types::J_matrix J_dot;
    J_dot.R = R_dot;
    J_dot.T = T_dot;

    return J_dot.as_matrix();
}

dp_types::Matrix6d calculate_C(double m,
                               const dp_types::Vector3d& r_b_bg,
                               const dp_types::Nu& nu_2,
                               const dp_types::Matrix3d& I_b) {
    dp_types::Matrix6d C;
    dp_types::Matrix3d C1 = m * skew_symmetric(nu_2.linear_speed);
    dp_types::Matrix3d C2 =
        -m * skew_symmetric(nu_2.angular_speed) * skew_symmetric(r_b_bg);
    dp_types::Matrix3d C3 =
        m * skew_symmetric(nu_2.angular_speed) * skew_symmetric(r_b_bg);
    dp_types::Matrix3d C4 = skew_symmetric(I_b * nu_2.angular_speed);

    C << C1, C2, C3, C4;

    return C;
}

dp_types::Matrix6x12d calculate_Y_v(const dp_types::Nu& nu) {
    dp_types::Matrix6x12d Y_v;
    Y_v.setZero();

    Y_v(0, 0) = nu.linear_speed.x();
    Y_v(0, 1) = nu.linear_speed.x() * std::abs(nu.linear_speed.x());

    Y_v(1, 2) = nu.linear_speed.y();
    Y_v(1, 3) = nu.linear_speed.y() * std::abs(nu.linear_speed.y());

    Y_v(2, 4) = nu.linear_speed.z();
    Y_v(2, 5) = nu.linear_speed.z() * std::abs(nu.linear_speed.z());

    Y_v(3, 6) = nu.angular_speed.x();
    Y_v(3, 7) = nu.angular_speed.x() * std::abs(nu.angular_speed.x());

    Y_v(4, 8) = nu.angular_speed.y();
    Y_v(4, 9) = nu.angular_speed.y() * std::abs(nu.angular_speed.y());

    Y_v(5, 10) = nu.angular_speed.z();
    Y_v(5, 11) = nu.angular_speed.z() * std::abs(nu.angular_speed.z());

    return Y_v;
}

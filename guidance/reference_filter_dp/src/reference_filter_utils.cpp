#include <reference_filter_dp/reference_filter_utils.hpp>

Matrix3d calculate_R(const Vector6d& eta) {
    const double roll = eta(3);
    const double pitch = eta(4);
    const double yaw = eta(5);

    const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

    return q.toRotationMatrix();
}

Matrix3d calculate_T(const Vector6d& eta) {
    Matrix3d T;
    double phi = eta(3);
    double theta = eta(4);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);

    // Manually added the transformation matrix from Fossen 2021 p.29 eq: 2.41
    T(0, 0) = 1;
    T(0, 1) = sphi * stheta / ctheta;
    T(0, 2) = cphi * stheta / ctheta;
    T(1, 0) = 0;
    T(1, 1) = cphi;
    T(1, 2) = -sphi;
    T(2, 0) = 0;
    T(2, 1) = sphi / ctheta;
    T(2, 2) = cphi / ctheta;

    return T;
}

Matrix6d calculate_J(const Vector6d& eta) {
    Matrix3d R = calculate_R(eta);
    Matrix3d T = calculate_T(eta);

    Matrix6d J = Matrix6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<3, 3>(3, 3) = T;

    return J;
}

double ssa(double angle) {
    double angle_ssa = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    return angle_ssa;
}

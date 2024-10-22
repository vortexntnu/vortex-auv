#include <reference_filter_dp/reference_filter_utils.hpp>

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &quat) {
    Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler_angles;
}

Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d &euler_angles) {
    Eigen::AngleAxisd rollAngle(euler_angles(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler_angles(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler_angles(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
    return quat;
}

Matrix3d calculate_R(const Vector6d &eta) {
    double phi = eta(3);
    double theta = eta(4);
    double psi = eta(5);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);
    double cpsi = cos(psi);
    double spsi = sin(psi);

    Matrix3d rotation_matrix;

    rotation_matrix << cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi,
                       spsi * ctheta, spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi,
                       -stheta, ctheta * sphi, ctheta * cphi;

    return rotation_matrix;
}

Matrix3d calculate_T(const Vector6d &eta) {
    double phi = eta(3);
    double theta = eta(4);

    double cphi = cos(phi);
    double sphi = sin(phi);
    double ctheta = cos(theta);
    double stheta = sin(theta);

    if (ctheta == 0) {
        throw std::runtime_error("Division by zero in calculate_T.");
    }

    Matrix3d transformation_matrix;

    transformation_matrix << 1, sphi * stheta / ctheta, cphi * stheta / ctheta,
                             0, cphi, -sphi,
                             0, sphi / ctheta, cphi / ctheta;

    return transformation_matrix;
}

Matrix6d calculate_J(const Vector6d &eta) {
    Matrix3d R = calculate_R(eta);
    Matrix3d T = calculate_T(eta);

    Matrix6d J = Matrix6d::Zero();
    J.block<3, 3>(0, 0) = R;
    J.block<3, 3>(3, 3) = T;

    return J;
}
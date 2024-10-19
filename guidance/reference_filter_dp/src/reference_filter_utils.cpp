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
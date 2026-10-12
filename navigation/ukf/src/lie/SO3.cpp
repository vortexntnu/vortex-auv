#include "ukf/lie/SO3.hpp"
#include <cmath>

namespace SO3 {

Eigen::Quaterniond Exp(const Eigen::Vector3d& rotation_vector) {
    double angle = rotation_vector.norm();
    Eigen::Quaterniond quaternion;

    if (angle < 1e-7) {
        // when angle is small utilize first order approximation
        quaternion.w() = 1;
        quaternion.x() = 0.5 * rotation_vector(0);
        quaternion.y() = 0.5 * rotation_vector(1);
        quaternion.z() = 0.5 * rotation_vector(2);
    } else {
        // otherwise utilize the exact mapping
        Eigen::Vector3d axis = rotation_vector / angle;
        quaternion.w() = cos(angle / 2);
        quaternion.x() = axis(0) * sin(angle / 2);
        quaternion.y() = axis(1) * sin(angle / 2);
        quaternion.z() = axis(2) * sin(angle / 2);
    }

    return quaternion.normalized();
}

Eigen::Vector3d Log(const Eigen::Quaterniond& quaternion) {
    // enforce one hemoshphere the quaternion lies on, to avoid double cover
    double sign = quaternion.w() < 0 ? -1.0 : 1.0;
    double real_part = sign * quaternion.w();

    Eigen::Vector3d hyperimaginary_part =
        sign * Eigen::Vector3d{quaternion.x(), quaternion.y(), quaternion.z()};

    double angle;
    Eigen::Vector3d axis;

    if (hyperimaginary_part.norm() < 1e-8) {
        // first order approximation
        angle = 1;
        axis = 2 * hyperimaginary_part;
    } else {
        // utilize the exact mapping
        angle = 2 * atan2(hyperimaginary_part.norm(), real_part);
        axis = hyperimaginary_part / hyperimaginary_part.norm();
    }
    return angle * axis;
}

}  // namespace SO3

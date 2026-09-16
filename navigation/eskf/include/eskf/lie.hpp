#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace eskf_math {
inline Eigen::Matrix3d skew(const Eigen::Vector3d& x) {
    Eigen::Matrix3d result;
    result << 0, -x.z(), x.y(), x.z(), 0, -x.x(), -x.y(), x.x(), 0;
    return result;
}
inline Eigen::Quaterniond exp(const Eigen::Vector3d& x) {
    const double angle = x.norm();
    const double scale = angle < 1e-6 ? 0.5 - angle * angle / 48.0
                                      : std::sin(angle / 2.0) / angle;
    return Eigen::Quaterniond(std::cos(angle / 2.0), scale * x.x(),
                              scale * x.y(), scale * x.z())
        .normalized();
}
// Derivative of Log(Exp(-x) Exp(x + epsilon)) at epsilon = 0.
inline Eigen::Matrix3d right_jacobian(const Eigen::Vector3d& x) {
    const double angle2 = x.squaredNorm();
    const double angle = std::sqrt(angle2);
    const double a = angle < 1e-4
                         ? 0.5 - angle2 / 24.0 + angle2 * angle2 / 720.0
                         : (1.0 - std::cos(angle)) / angle2;
    const double b = angle < 1e-4
                         ? 1.0 / 6.0 - angle2 / 120.0 + angle2 * angle2 / 5040.0
                         : (angle - std::sin(angle)) / (angle2 * angle);
    const auto cross = skew(x);
    return Eigen::Matrix3d::Identity() - a * cross + b * cross * cross;
}
}  // namespace eskf_math

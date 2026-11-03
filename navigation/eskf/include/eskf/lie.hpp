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
}  // namespace eskf_math


#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
}

double sq(const double& value) {
    return value * value;
}

Eigen::Quaterniond vector3d_to_quaternion(const Eigen::Vector3d& vector) {
    double angle = vector.norm();
    if (angle < 1e-8) {
        return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    } else {
        Eigen::Vector3d axis = vector / angle;
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    }
}

Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& euler) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    return q;
}

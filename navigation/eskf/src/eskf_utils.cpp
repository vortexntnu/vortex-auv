
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
}

double sq(double value) {
    return value * value;
}
double ssa(double angle) {
    double result = fmod(angle + M_PI, 2 * M_PI);
    double angle_ssa = result < 0 ? result + M_PI : result - M_PI;
    return angle_ssa;
}

Eigen::Matrix4x3d calculate_T_q(const Eigen::Quaterniond& quat) {
    Eigen::Matrix4x3d T_q = Eigen::Matrix4x3d::Zero();
    double qw = quat.w();
    double qx = quat.x();
    double qy = quat.y();
    double qz = quat.z();

    T_q << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;

    T_q *= 0.5;
    return T_q;
}

Eigen::Quaterniond vector3d_to_quaternion(const Eigen::Vector3d& vector) {
    double angle = vector.norm();
    if (angle < 1e-8) {
        return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    } else {
        Eigen::Vector3d axis = vector / angle;
        Eigen::Quaterniond quat =
            Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        return quat.normalized();
    }
}

Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& euler) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    q.normalize();
    return q;
}

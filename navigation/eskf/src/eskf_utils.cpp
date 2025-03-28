
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

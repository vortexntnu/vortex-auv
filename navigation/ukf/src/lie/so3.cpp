#include "ukf/lie/so3.hpp"

namespace so3 {

Eigen::Quaterniond exp(const Eigen::Vector3d& rotation_vector) {
    // TODO: exact exp, with a series expansion near zero
    return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d log(const Eigen::Quaterniond& quaternion) {
    // TODO: exact log, see error_quaternion in vortex utils for the sign
    return Eigen::Vector3d::Zero();
}

}  // namespace so3

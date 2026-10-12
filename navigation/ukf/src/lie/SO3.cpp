#include "ukf/lie/SO3.hpp"

namespace SO3 {

Eigen::Quaterniond Exp(const Eigen::Vector3d& rotation_vector) {
    // TODO: exact exp, with a series expansion near zero
    return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d Log(const Eigen::Quaterniond& quaternion) {
    // TODO: exact log, see error_quaternion in vortex utils for the sign
    return Eigen::Vector3d::Zero();
}

}  // namespace SO3

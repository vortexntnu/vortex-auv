#ifndef UKF__LIE__SO3_HPP_
#define UKF__LIE__SO3_HPP_

#include <Eigen/Geometry>

namespace so3 {

Eigen::Quaterniond exp(const Eigen::Vector3d& rotation_vector);

Eigen::Vector3d log(const Eigen::Quaterniond& quaternion);

}  // namespace so3

#endif  // UKF__LIE__SO3_HPP_

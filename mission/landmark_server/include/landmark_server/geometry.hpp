#ifndef LANDMARK_SERVER__GEOMETRY_HPP_
#define LANDMARK_SERVER__GEOMETRY_HPP_

#include <Eigen/Geometry>

#include <cmath>

namespace vortex::landmark_server {

using Pose = Eigen::Isometry3d;

inline double yaw_of(const Eigen::Matrix3d& R) {
    return std::atan2(R(1, 0), R(0, 0));
}

inline double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

inline Pose yaw_pose(double yaw, const Eigen::Vector3d& p) {
    Pose T = Pose::Identity();
    T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
    T.translation() = p;
    return T;
}

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__GEOMETRY_HPP_

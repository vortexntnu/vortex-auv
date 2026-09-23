#ifndef LANDMARK_TARGETS_TEST_UTILS_HPP_
#define LANDMARK_TARGETS_TEST_UTILS_HPP_

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <vortex/utils/types.hpp>

namespace vortex::mission::test {

inline vortex::utils::types::Pose make_pose(double x,
                                            double y,
                                            double z,
                                            double roll = 0.0,
                                            double pitch = 0.0,
                                            double yaw = 0.0) {
    const Eigen::Quaterniond q =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return vortex::utils::types::Pose::from_eigen(Eigen::Vector3d(x, y, z), q);
}

inline double yaw_of(const vortex::utils::types::Pose& p) {
    const Eigen::Quaterniond q = p.ori_quaternion();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace vortex::mission::test

#endif  // LANDMARK_TARGETS_TEST_UTILS_HPP_

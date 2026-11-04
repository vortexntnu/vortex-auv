#ifndef LANDMARK_SERVER__DETECTION_CHECK_HPP_
#define LANDMARK_SERVER__DETECTION_CHECK_HPP_

#include <Eigen/Geometry>

#include <optional>

#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace vortex::landmark_server {

struct CheckedPose {
    Eigen::Vector3d position;
    /// Empty when the detector gave no usable orientation.
    std::optional<Eigen::Quaterniond> rotation;
};

/// Empty for a NaN position or a given orientation that is not a unit
/// quaternion. All-zero covariance means none given: no orientation.
std::optional<CheckedPose> check_detection(
    const geometry_msgs::msg::PoseWithCovariance& pose);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__DETECTION_CHECK_HPP_

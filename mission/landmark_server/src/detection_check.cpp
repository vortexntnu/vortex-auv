#include "landmark_server/detection_check.hpp"

#include <algorithm>
#include <cmath>

namespace vortex::landmark_server {

namespace {

constexpr double kNoOrientationVariance = 1000.0;  // perception convention
constexpr double kUnitTolerance = 0.05;

}  // namespace

std::optional<CheckedPose> check_detection(
    const geometry_msgs::msg::PoseWithCovariance& pose) {
    const auto& p = pose.pose.position;
    CheckedPose out;
    out.position = {p.x, p.y, p.z};
    if (!out.position.allFinite()) {
        return std::nullopt;
    }
    const auto& c = pose.covariance;
    const bool all_zero =
        std::all_of(c.begin(), c.end(), [](double v) { return v == 0.0; });
    if (all_zero ||
        !(c[21] < kNoOrientationVariance && c[28] < kNoOrientationVariance &&
          c[35] < kNoOrientationVariance)) {
        return out;
    }
    const auto& o = pose.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    const double norm = q.norm();
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > kUnitTolerance) {
        return std::nullopt;
    }
    out.rotation = q.normalized();
    return out;
}

}  // namespace vortex::landmark_server

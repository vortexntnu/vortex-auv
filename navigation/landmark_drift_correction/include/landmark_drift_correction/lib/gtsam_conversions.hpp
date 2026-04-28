#ifndef LANDMARK_DRIFT_CORRECTION__LIB__GTSAM_CONVERSIONS_HPP_
#define LANDMARK_DRIFT_CORRECTION__LIB__GTSAM_CONVERSIONS_HPP_

#include <gtsam/geometry/Pose3.h>
#include <vector>
#include "typedefs.hpp"

namespace vortex::navigation {

inline gtsam::Pose3 to_gtsam_pose(const Eigen::Vector3d& translation,
                                  const Eigen::Quaterniond& rotation) {
    return gtsam::Pose3(gtsam::Rot3(rotation.toRotationMatrix()),
                        gtsam::Point3(translation));
}

inline Vector6d vector6d_to_eigen(const std::vector<double>& values) {
    if (values.size() != 6) {
        throw std::invalid_argument(
            "Vector must contain exactly 6 values to convert to "
            "Eigen::Vector6d, got " +
            std::to_string(values.size()));
    }

    return Eigen::Map<const Vector6d>(values.data());
}

}  // namespace vortex::navigation

#endif  // LANDMARK_DRIFT_CORRECTION__LIB__GTSAM_CONVERSIONS_HPP_

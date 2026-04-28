#ifndef LANDMARK_DRIFT_CORRECTION__LIB__TYPEDEFS_HPP_
#define LANDMARK_DRIFT_CORRECTION__LIB__TYPEDEFS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

namespace vortex::navigation {

using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief Represents a keyframe in the drift correction graph.
 *
 * A keyframe stores the graph key, timestamp, and pose estimate for one
 * pose node in the drift correction factor graph.
 *
 * Members:
 * - `k`: GTSAM graph key index `X(k)`. `X(0)` is the prior anchor.
 * - `t`: Timestamp in seconds.
 * - `translation`: Keyframe position.
 * - `rotation`: Keyframe orientation.
 */
struct Keyframe {
    std::size_t k;  ///< GTSAM graph key index X(k); X(0) is the prior anchor.
    double t;       ///< Timestamp [s].
    Eigen::Vector3d translation;  ///< Keyframe position.
    Eigen::Quaterniond rotation;  ///< Keyframe orientation.
};

}  // namespace vortex::navigation

#endif  // LANDMARK_DRIFT_CORRECTION__LIB__TYPEDEFS_HPP_

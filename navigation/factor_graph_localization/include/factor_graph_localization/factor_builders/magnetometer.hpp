#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__MAGNETOMETER_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__MAGNETOMETER_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/MagPoseFactor.h>

namespace vortex::navigation {

struct MagnetometerMeasurement {
    /// Calibrated field in MAG axes [T], with bridge-offset and
    /// hard/soft-iron corrections applied.
    gtsam::Vector3 field_MAG_tesla;
    /// Effective field covariance in calibrated MAG axes [T^2].
    gtsam::Matrix3 covariance_MAG_tesla2;
};

/**
 * IMU_pose_key: IMU Pose3 graph variable at the measurement timestamp,
 * expressed in the fixed STARTUP reference with z down and startup yaw zero.
 * IMU_R_MAG: MAG -> IMU mounting rotation.
 * field_STARTUP_tesla: fixed nonzero reference field [T], uniform over the
 * mission.
 */
gtsam::MagPoseFactor<gtsam::Pose3> build_magnetometer_factor(
    const gtsam::Key IMU_pose_key,
    const MagnetometerMeasurement& magnetometer_measurement,
    const gtsam::Rot3& IMU_R_MAG,
    const gtsam::Vector3& field_STARTUP_tesla);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__MAGNETOMETER_HPP_

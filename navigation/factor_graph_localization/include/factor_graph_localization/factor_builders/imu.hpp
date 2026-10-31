#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__IMU_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__IMU_HPP_

#include <gtsam/inference/Key.h>
#include <gtsam/navigation/ImuFactor.h>

namespace vortex::navigation {

/**
 * preint_start/end: graph states bounding the preintegration interval.
 * Pose keys: IMU poses in GTSAM's fixed reference frame.
 * Velocity keys: IMU-origin velocities in that fixed frame [m/s].
 * IMU bias key: accelerometer [m/s^2] and gyro [rad/s] biases at preint_start,
 * expressed in IMU axes.
 * preintegrated_measurements: accumulated IMU measurements and covariance.
 */
gtsam::ImuFactor build_imu_factor(
    const gtsam::Key preint_start_pose_key,
    const gtsam::Key preint_start_velocity_key,
    const gtsam::Key preint_end_pose_key,
    const gtsam::Key preint_end_velocity_key,
    const gtsam::Key preint_start_IMU_bias_key,
    const gtsam::PreintegratedImuMeasurements& preintegrated_measurements);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__IMU_HPP_

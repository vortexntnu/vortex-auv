#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__IMU_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__IMU_HPP_

#include <gtsam/inference/Key.h>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace vortex::navigation {

/**
 * preint_start/end: graph states bounding the preintegration interval.
 * Pose keys: IMU poses in GTSAM's fixed WORLD reference frame.
 * Velocity keys: IMU-origin velocities in WORLD axes [m/s].
 * IMU bias keys: accelerometer [m/s^2] and gyro [rad/s] biases at both
 * interval endpoints, expressed in IMU axes.
 * preintegrated_measurements: accumulated IMU measurements and covariance.
 */
gtsam::CombinedImuFactor build_combined_imu_factor(
    const gtsam::Key preint_start_IMU_pose_key,
    const gtsam::Key preint_start_WORLD_velocity_key,
    const gtsam::Key preint_end_IMU_pose_key,
    const gtsam::Key preint_end_WORLD_velocity_key,
    const gtsam::Key preint_start_IMU_bias_key,
    const gtsam::Key preint_end_IMU_bias_key,
    const gtsam::PreintegratedCombinedMeasurements& preintegrated_measurements);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__IMU_HPP_

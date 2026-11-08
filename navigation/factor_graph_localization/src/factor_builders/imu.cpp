#include "factor_graph_localization/factor_builders/imu.hpp"

namespace vortex::navigation {

gtsam::CombinedImuFactor build_combined_imu_factor(
    const gtsam::Key preint_start_IMU_pose_key,
    const gtsam::Key preint_start_WORLD_velocity_key,
    const gtsam::Key preint_end_IMU_pose_key,
    const gtsam::Key preint_end_WORLD_velocity_key,
    const gtsam::Key preint_start_IMU_bias_key,
    const gtsam::Key preint_end_IMU_bias_key,
    const gtsam::PreintegratedCombinedMeasurements& preintegrated_measurements) {
    return gtsam::CombinedImuFactor(
        preint_start_IMU_pose_key, preint_start_WORLD_velocity_key,
        preint_end_IMU_pose_key, preint_end_WORLD_velocity_key,
        preint_start_IMU_bias_key, preint_end_IMU_bias_key,
        preintegrated_measurements);
}

}  // namespace vortex::navigation

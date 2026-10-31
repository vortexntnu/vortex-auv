#include "factor_graph_localization/factor_builders/imu.hpp"

namespace vortex::navigation {

gtsam::ImuFactor build_imu_factor(
    const gtsam::Key preint_start_pose_key,
    const gtsam::Key preint_start_velocity_key,
    const gtsam::Key preint_end_pose_key,
    const gtsam::Key preint_end_velocity_key,
    const gtsam::Key preint_start_IMU_bias_key,
    const gtsam::PreintegratedImuMeasurements& preintegrated_measurements) {
    return gtsam::ImuFactor(
        preint_start_pose_key, preint_start_velocity_key,
        preint_end_pose_key, preint_end_velocity_key,
        preint_start_IMU_bias_key, preintegrated_measurements);
}

}  // namespace vortex::navigation

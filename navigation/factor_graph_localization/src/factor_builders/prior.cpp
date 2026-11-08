#include "factor_graph_localization/factor_builders/prior.hpp"

#include <gtsam/linear/NoiseModel.h>

namespace vortex::navigation {

gtsam::PriorFactor<gtsam::Pose3> build_pose_prior_factor(
    const gtsam::Key IMU_pose_key,
    const gtsam::Pose3& WORLD_P_IMU,
    const gtsam::Matrix6& pose_covariance_rot_pos) {
    return gtsam::PriorFactor<gtsam::Pose3>(
        IMU_pose_key, WORLD_P_IMU,
        gtsam::noiseModel::Gaussian::Covariance(pose_covariance_rot_pos));
}

gtsam::PriorFactor<gtsam::Vector3> build_velocity_prior_factor(
    const gtsam::Key IMU_origin_vel_WORLD_key,
    const gtsam::Vector3& IMU_origin_vel_WORLD_mps,
    const gtsam::Matrix3& velocity_covariance_WORLD_m2ps2) {
    return gtsam::PriorFactor<gtsam::Vector3>(
        IMU_origin_vel_WORLD_key, IMU_origin_vel_WORLD_mps,
        gtsam::noiseModel::Gaussian::Covariance(
            velocity_covariance_WORLD_m2ps2));
}

gtsam::PriorFactor<gtsam::imuBias::ConstantBias> build_imu_bias_prior_factor(
    const gtsam::Key IMU_bias_key,
    const gtsam::imuBias::ConstantBias& bias_IMU,
    const gtsam::Matrix6& bias_covariance_accel_gyro_IMU) {
    return gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        IMU_bias_key, bias_IMU,
        gtsam::noiseModel::Gaussian::Covariance(
            bias_covariance_accel_gyro_IMU));
}

}  // namespace vortex::navigation

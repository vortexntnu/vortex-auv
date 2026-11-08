#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__PRIOR_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__PRIOR_HPP_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/PriorFactor.h>

namespace vortex::navigation {

gtsam::PriorFactor<gtsam::Pose3> build_pose_prior_factor(
    const gtsam::Key IMU_pose_key,
    const gtsam::Pose3& WORLD_P_IMU,
    const gtsam::Matrix6& pose_covariance_rot_pos);

gtsam::PriorFactor<gtsam::Vector3> build_velocity_prior_factor(
    const gtsam::Key IMU_origin_vel_WORLD_key,
    const gtsam::Vector3& IMU_origin_vel_WORLD_mps,
    const gtsam::Matrix3& velocity_covariance_WORLD_m2ps2);

gtsam::PriorFactor<gtsam::imuBias::ConstantBias> build_imu_bias_prior_factor(
    const gtsam::Key IMU_bias_key,
    const gtsam::imuBias::ConstantBias& bias_IMU,
    const gtsam::Matrix6& bias_covariance_accel_gyro_IMU);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__PRIOR_HPP_

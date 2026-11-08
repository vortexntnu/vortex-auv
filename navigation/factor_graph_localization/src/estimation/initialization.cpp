#include "factor_graph_localization/estimation/initialization.hpp"

#include "factor_graph_localization/factor_builders/prior.hpp"

namespace vortex::navigation {

InitialState make_initial_state_from_startup(
    const double timestamp_s,
    const StartupMagnetometerReference& startup_reference,
    const gtsam::Point3& IMU_origin_pos_WORLD_m,
    const gtsam::Vector3& IMU_origin_vel_IMU_mps,
    const gtsam::imuBias::ConstantBias& bias_IMU) {
    return {timestamp_s,
            gtsam::Pose3(startup_reference.WORLD_R_IMU_start,
                         IMU_origin_pos_WORLD_m),
            IMU_origin_vel_IMU_mps,
            bias_IMU,
            startup_reference.field_WORLD_tesla};
}

InitialGraph make_initial_graph(
    const gtsam::Key IMU_pose_key,
    const gtsam::Key IMU_origin_vel_WORLD_key,
    const gtsam::Key IMU_bias_key,
    const InitialState& initial_state,
    const InitialPriorCovariances& prior_covariances) {
    const gtsam::Vector3 IMU_origin_vel_WORLD_mps =
        initial_state.WORLD_P_IMU.rotation().rotate(
            initial_state.IMU_origin_vel_IMU_mps);

    InitialGraph initial_graph{initial_state.timestamp_s, {}, {},
                               initial_state.field_WORLD_tesla};
    initial_graph.factors.add(build_pose_prior_factor(
        IMU_pose_key, initial_state.WORLD_P_IMU,
        prior_covariances.pose_covariance_rot_pos));
    initial_graph.factors.add(build_velocity_prior_factor(
        IMU_origin_vel_WORLD_key, IMU_origin_vel_WORLD_mps,
        prior_covariances.velocity_covariance_WORLD_m2ps2));
    initial_graph.factors.add(build_imu_bias_prior_factor(
        IMU_bias_key, initial_state.bias_IMU,
        prior_covariances.bias_covariance_accel_gyro_IMU));

    initial_graph.values.insert(IMU_pose_key, initial_state.WORLD_P_IMU);
    initial_graph.values.insert(IMU_origin_vel_WORLD_key,
                                IMU_origin_vel_WORLD_mps);
    initial_graph.values.insert(IMU_bias_key, initial_state.bias_IMU);
    return initial_graph;
}

}  // namespace vortex::navigation

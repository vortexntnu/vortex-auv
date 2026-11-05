#include "factor_graph_localization/factor_builders/dvl_beam.hpp"

#include <gtsam/linear/NoiseModel.h>

#include "factor_graph_localization/expressions/dvl_beam.hpp"

namespace vortex::navigation {

gtsam::ExpressionFactor<double> build_dvl_beam_factor(
    const gtsam::Key IMU_pose_key,
    const gtsam::Key IMU_origin_vel_WORLD_key,
    const gtsam::Key IMU_bias_key,
    const DvlBeamMeasurement& beam_measurement,
    const DvlBeamGeometry& beam_geometry,
    const gtsam::Vector3& measured_angular_vel_IMU) {
    const auto& measured_beam_vel_mps = beam_measurement.vel_mps;
    const auto& sigma_b = beam_measurement.sigma_mps;

    const gtsam::Pose3_ WORLD_P_IMU(IMU_pose_key);
    const gtsam::Vector3_ vel_WORLD(IMU_origin_vel_WORLD_key);
    const ImuBias_ estimated_accel_gyro_bias_IMU(IMU_bias_key);

    const auto h = predict_dvl_beam_vel(
        WORLD_P_IMU, vel_WORLD, estimated_accel_gyro_bias_IMU,
        beam_geometry.IMU_P_DVL, measured_angular_vel_IMU,
        beam_geometry.beam_direction);

    const auto beam_vel_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_b);

    return gtsam::ExpressionFactor<double>(beam_vel_noise_model,
                                           measured_beam_vel_mps, h);
}

}  // namespace vortex::navigation

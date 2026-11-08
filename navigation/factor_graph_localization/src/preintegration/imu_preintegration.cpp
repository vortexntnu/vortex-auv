#include "factor_graph_localization/preintegration/imu_preintegration.hpp"

#include <gtsam/navigation/PreintegrationCombinedParams.h>

namespace vortex::navigation {
namespace {

gtsam::PreintegratedCombinedMeasurements make_measurements(
    const ImuPreintegrationParameters& parameters,
    const gtsam::imuBias::ConstantBias& bias_IMU) {
    // Fixed WORLD frame, +Z points down. Hence, MakeSharedD
    auto params =
        gtsam::PreintegrationCombinedParams::MakeSharedD(parameters.gravity_m_s2);

    params->setAccelerometerCovariance(parameters.accelerometer_covariance_IMU);

    params->setGyroscopeCovariance(parameters.gyroscope_covariance_IMU);

    params->setBiasAccCovariance(
        parameters.accelerometer_bias_random_walk_covariance_IMU);

    params->setBiasOmegaCovariance(
        parameters.gyroscope_bias_random_walk_covariance_IMU);

    //this is often zero or 1e-8 magnitude
    params->setIntegrationCovariance(parameters.integration_covariance);

    return gtsam::PreintegratedCombinedMeasurements(params, bias_IMU);
}

}  // namespace

ImuPreintegration::ImuPreintegration(
    const ImuPreintegrationParameters& parameters,
    const gtsam::imuBias::ConstantBias& bias_IMU)
    : measurements_(make_measurements(parameters, bias_IMU)) {}

void ImuPreintegration::integrate_measurement(
    const gtsam::Vector3& specific_force_IMU_m_s2,
    const gtsam::Vector3& angular_velocity_IMU_rad_s,
    const double imu_sample_interval_s) {
    measurements_.integrateMeasurement(specific_force_IMU_m_s2,
                                       angular_velocity_IMU_rad_s,
                                       imu_sample_interval_s);
}

void ImuPreintegration::reset_integration(
    const gtsam::imuBias::ConstantBias& bias_IMU) {
    measurements_.resetIntegrationAndSetBias(bias_IMU);
}

const gtsam::PreintegratedCombinedMeasurements& ImuPreintegration::measurements()
    const {
    return measurements_;
}

}  // namespace vortex::navigation

#ifndef FACTOR_GRAPH_LOCALIZATION__PREINTEGRATION__IMU_PREINTEGRATION_HPP_
#define FACTOR_GRAPH_LOCALIZATION__PREINTEGRATION__IMU_PREINTEGRATION_HPP_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

namespace vortex::navigation {

struct ImuPreintegrationParameters {
    double gravity_m_s2{0.0};

    /// [(m/s^2)^2/Hz].
    gtsam::Matrix3 accelerometer_covariance_IMU{gtsam::Matrix3::Zero()};

    /// [(rad/s)^2/Hz].
    gtsam::Matrix3 gyroscope_covariance_IMU{gtsam::Matrix3::Zero()};

    /// [(m/s^2)^2/s].
    gtsam::Matrix3 accelerometer_bias_random_walk_covariance_IMU{
        gtsam::Matrix3::Zero()};

    /// [(rad/s)^2/s].
    gtsam::Matrix3 gyroscope_bias_random_walk_covariance_IMU{
        gtsam::Matrix3::Zero()};

    /// [m^2/s].
    gtsam::Matrix3 integration_covariance{gtsam::Matrix3::Zero()};
};

class ImuPreintegration {
   public:
    ImuPreintegration(const ImuPreintegrationParameters& parameters,
                      const gtsam::imuBias::ConstantBias& bias_IMU);

    void integrate_measurement(
        const gtsam::Vector3& specific_force_IMU_m_s2,
        const gtsam::Vector3& angular_velocity_IMU_rad_s,
        const double imu_sample_interval_s);

    void reset_integration(const gtsam::imuBias::ConstantBias& bias_IMU);

    const gtsam::PreintegratedCombinedMeasurements& measurements() const;

   private:
    gtsam::PreintegratedCombinedMeasurements measurements_;
};

}

#endif

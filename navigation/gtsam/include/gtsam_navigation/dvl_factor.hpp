#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam_navigation {

/** Bottom-track velocity measured at the DVL origin in DVL axes.
 * Bias variables and the sampled gyro are expressed in IMU axes.
 */
class DvlFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Pose3,
                                      gtsam::Vector3,
                                      gtsam::imuBias::ConstantBias> {
   public:
    DvlFactor(gtsam::Key pose,
              gtsam::Key velocity,
              gtsam::Key bias,
              const gtsam::Vector3& measurement,
              const gtsam::Vector3& gyro,
              const gtsam::Pose3& imu_p_dvl,
              const gtsam::SharedNoiseModel& noise);

    using Base = gtsam::NoiseModelFactor3<gtsam::Pose3,
                                          gtsam::Vector3,
                                          gtsam::imuBias::ConstantBias>;
    using Base::evaluateError;
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector3& velocity,
        const gtsam::imuBias::ConstantBias& bias,
        gtsam::OptionalMatrixType h_pose = nullptr,
        gtsam::OptionalMatrixType h_velocity = nullptr,
        gtsam::OptionalMatrixType h_bias = nullptr) const override;

   private:
    gtsam::Vector3 measurement_, gyro_;
    gtsam::Pose3 imu_p_dvl_;
};
}  // namespace gtsam_navigation

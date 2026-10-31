#include "factor_graph_localization/factor_builders/magnetometer.hpp"

#include <gtsam/linear/NoiseModel.h>

namespace vortex::navigation {

gtsam::MagPoseFactor<gtsam::Pose3> build_magnetometer_factor(
    const gtsam::Key IMU_pose_key,
    const MagnetometerMeasurement& magnetometer_measurement,
    const gtsam::Rot3& IMU_R_MAG,
    const gtsam::Vector3& field_STARTUP_tesla) {
    const gtsam::Vector3 field_IMU_tesla =
        IMU_R_MAG.rotate(magnetometer_measurement.field_MAG_tesla);

    const gtsam::Matrix3 IMU_R_MAG_matrix = IMU_R_MAG.matrix();

    const gtsam::Matrix3 covariance_IMU_tesla2 =
        IMU_R_MAG_matrix * magnetometer_measurement.covariance_MAG_tesla2 *
        IMU_R_MAG_matrix.transpose();

    const auto field_noise_model =
        gtsam::noiseModel::Gaussian::Covariance(covariance_IMU_tesla2);

    // MagPoseFactor normalizes direction; scale retains the field magnitude.
    return gtsam::MagPoseFactor<gtsam::Pose3>(
        IMU_pose_key, field_IMU_tesla, field_STARTUP_tesla.norm(),
        field_STARTUP_tesla, gtsam::Vector3::Zero(), field_noise_model, {});
}

}  // namespace vortex::navigation

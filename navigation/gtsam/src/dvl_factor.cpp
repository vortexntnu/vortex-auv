#include "gtsam_navigation/dvl_factor.hpp"

namespace gtsam_navigation {
DvlFactor::DvlFactor(gtsam::Key pose,
                     gtsam::Key velocity,
                     gtsam::Key bias,
                     const gtsam::Vector3& measurement,
                     const gtsam::Vector3& gyro,
                     const gtsam::Pose3& imu_p_dvl,
                     const gtsam::SharedNoiseModel& noise)
    : Base(noise, pose, velocity, bias),
      measurement_(measurement),
      gyro_(gyro),
      imu_p_dvl_(imu_p_dvl) {}

gtsam::Vector DvlFactor::evaluateError(const gtsam::Pose3& pose,
                                       const gtsam::Vector3& velocity,
                                       const gtsam::imuBias::ConstantBias& bias,
                                       gtsam::OptionalMatrixType h_pose,
                                       gtsam::OptionalMatrixType h_velocity,
                                       gtsam::OptionalMatrixType h_bias) const {
    const auto r_db = imu_p_dvl_.rotation().matrix().transpose().eval();
    const auto r_bw = pose.rotation().matrix().transpose().eval();
    const gtsam::Vector3 v_body = r_bw * velocity;
    const gtsam::Vector3 omega = gyro_ - bias.gyroscope();
    if (h_pose) {
        *h_pose = gtsam::Matrix::Zero(3, 6);
        h_pose->leftCols<3>() = r_db * gtsam::skewSymmetric(v_body);
    }
    if (h_velocity) {
        *h_velocity = r_db * r_bw;
    }
    if (h_bias) {
        *h_bias = gtsam::Matrix::Zero(3, 6);
        h_bias->rightCols<3>() =
            r_db * gtsam::skewSymmetric(imu_p_dvl_.translation());
    }
    return r_db * (v_body + omega.cross(imu_p_dvl_.translation())) -
           measurement_;
}
}  // namespace gtsam_navigation

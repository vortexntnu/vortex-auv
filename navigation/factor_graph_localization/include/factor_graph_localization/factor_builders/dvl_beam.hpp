#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace vortex::navigation {

struct DvlBeamMeasurement {
    double vel_mps;
    /// DVL beam standard deviation [m/s].
    double sigma_mps;
};

struct DvlBeamGeometry {
    /// Outward beam direction in DVL axes.
    gtsam::Unit3 beam_direction;
    /// DVL -> IMU transform using the DVL measurement reference origin.
    /// Translation: IMU -> DVL origin, in IMU axes [m].
    gtsam::Pose3 IMU_P_DVL;
};

/**
 * IMU_pose_key: Pose3 mapping the IMU sensing frame into fixed WORLD axes.
 * IMU_origin_vel_WORLD_key: IMU-origin velocity in WORLD axes [m/s].
 * IMU_bias_key: accelerometer/gyro bias state in IMU axes.
 * measured_angular_vel_IMU: gyro measurement [rad/s].
 */
gtsam::ExpressionFactor<double> build_dvl_beam_factor(
    const gtsam::Key IMU_pose_key,
    const gtsam::Key IMU_origin_vel_WORLD_key,
    const gtsam::Key IMU_bias_key,
    const DvlBeamMeasurement& beam_measurement,
    const DvlBeamGeometry& beam_geometry,
    const gtsam::Vector3& measured_angular_vel_IMU);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_

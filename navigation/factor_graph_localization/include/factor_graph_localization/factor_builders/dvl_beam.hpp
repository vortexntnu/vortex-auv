#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace vortex::navigation {

struct DvlBeamMeasurement {
    double vel_mps;
    /// Positive DVL beam velocity standard deviation [m/s].
    double sigma_mps;
};

struct DvlBeamGeometry {
    /// Outward beam direction in DVL axes, configured in sensors.yaml.
    gtsam::Unit3 beam_direction;
    /// DVL -> IMU transform using the DVL measurement reference origin.
    /// Translation: IMU -> DVL origin, in IMU axes [m].
    gtsam::Pose3 IMU_P_DVL;
};

/**
 * IMU_pose_key: Pose3 mapping the IMU sensing frame into fixed STARTUP axes.
 * IMU_origin_vel_STARTUP_key: velocity of the IMU origin relative to the
 * stationary bottom, expressed in STARTUP axes [m/s], as in GTSAM's IMU factor.
 * Both keys must describe the state at the accepted beam timestamp.
 * beam_measurement: measured beam velocity and effective standard deviation.
 * beam_geometry: fixed mounting transform and beam direction.
 * angular_vel_IMU: corresponding bias-corrected gyro measurement [rad/s].
 *
 * Returns a Gaussian beam-velocity factor connected to pose and velocity.
 * The IMU remains the navigation sensing frame and estimated origin.
 * Timestamp association and validity of measurements, noise, and geometry
 * belong to the caller; this builder performs no additional validation.
 */
gtsam::ExpressionFactor<double> build_dvl_beam_factor(
    const gtsam::Key IMU_pose_key,
    const gtsam::Key IMU_origin_vel_STARTUP_key,
    const DvlBeamMeasurement& beam_measurement,
    const DvlBeamGeometry& beam_geometry,
    const gtsam::Vector3& angular_vel_IMU);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_

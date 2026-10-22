#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace vortex::navigation {

struct DvlBeamMeasurement {
    double vel_mps;
    /// Positive Nucleus beam Figure Of Merit (standard deviation).
    double sigma_mps;
};

struct DvlBeamGeometry {
    /// Outward beam direction in DVL axes, configured in sensors.yaml.
    gtsam::Unit3 beam_direction;
    /// DVL -> STIM300 transform using Nucleus's virtual origin 
    /// STIM300 -> DVL origin, in STIM300 axes [m].
    gtsam::Pose3 IMU_P_DVL;
};

/**
 * IMU_origin_vel: Vector3 state at the STIM300 origin, in STIM300 axes,
 * relative to the stationary bottom [m/s].
 * angular_vel: bias-corrected gyro in STIM300 axes [rad/s].
 *
 * Mounting and gyro uncertainty are not propagated.
 * Inputs must be valid and aligned to the beam's measurement time.
 */
gtsam::ExpressionFactor<double> build_dvl_beam_factor(
    const gtsam::Key IMU_origin_vel,
    const DvlBeamMeasurement& beam_measurement,
    const DvlBeamGeometry& beam_geometry,
    const gtsam::Vector3& angular_vel);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__DVL_BEAM_HPP_

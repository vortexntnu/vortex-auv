#ifndef FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__PRESSURE_HPP_
#define FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__PRESSURE_HPP_

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

namespace vortex::navigation {

struct PressureMeasurement {
    /// Temperature-compensated absolute pressure from PressureSensor [Pa].
    double pressure_pa;
    /// Positive effective pressure standard deviation [Pa].
    double sigma_pa;
};

struct PressureGeometry {
    /// IMU -> PressureSensor port offset, in IMU axes [m].
    gtsam::Point3 IMU_PORT_OFFSET;
};

struct PressureModel {
    /// Atmospheric absolute pressure at the water surface [Pa].
    double surface_pressure_pa;
    /// Positive, constant water density [kg/m^3].
    double water_density_kg_m3;
    /// Positive gravitational acceleration [m/s^2].
    double gravity_mps2;
};

/**
 * IMU_pose_key: IMU Pose3 graph variable at the measurement timestamp.
 * The graph's fixed reference frame has z down and z = 0 at the water surface.
 */
gtsam::ExpressionFactor<double> build_pressure_factor(
    const gtsam::Key IMU_pose_key,
    const PressureMeasurement& pressure_measurement,
    const PressureGeometry& pressure_geometry,
    const PressureModel& pressure_model);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__FACTOR_BUILDERS__PRESSURE_HPP_

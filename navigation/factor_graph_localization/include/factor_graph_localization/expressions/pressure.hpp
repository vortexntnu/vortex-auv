#ifndef FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__PRESSURE_HPP_
#define FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__PRESSURE_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/expressions.h>

namespace vortex::navigation {

/**
 * @brief Hydrostatic pressure prediction h(IMU_pose) [Pa].
 *
 * IMU_pose: IMU pose in the graph's fixed reference frame, with z down
 * and z = 0 at the water surface.
 * IMU_PORT_OFFSET: IMU -> PressureSensor port offset in IMU axes [m].
 * surface_pressure_pa: atmospheric absolute pressure at the water surface [Pa].
 * water_density_kg_m3: constant water density [kg/m^3].
 * gravity_mps2: gravitational acceleration magnitude [m/s^2].
 */
inline gtsam::Expression<double> predict_pressure(
    const gtsam::Pose3_& IMU_pose,
    const gtsam::Point3& IMU_PORT_OFFSET,
    const double surface_pressure_pa,
    const double water_density_kg_m3,
    const double gravity_mps2) {
    const auto p_surface = gtsam::Double_(surface_pressure_pa);
    const auto rho = water_density_kg_m3;
    const auto g = gravity_mps2;
    const auto down = gtsam::Vector3_(gtsam::Vector3::UnitZ());
    const auto port_offset = gtsam::Point3_(IMU_PORT_OFFSET);

    const auto port_position = gtsam::transformFrom(IMU_pose, port_offset);
    const auto port_depth_m = gtsam::dot(down, port_position);

    return p_surface + (rho * g) * port_depth_m;
}

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__PRESSURE_HPP_

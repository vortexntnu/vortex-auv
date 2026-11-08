#ifndef FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__PRESSURE_HPP_
#define FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__PRESSURE_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/expressions.h>

namespace vortex::navigation {

/**
 * @brief Hydrostatic pressure prediction h(IMU_pose) [Pa].
 *
 * IMU_pose: IMU pose in the fixed WORLD frame, with z down
 * and z = 0 at the water surface.
 * pressure_port_offset_IMU_m: IMU -> PressureSensor port offset in IMU axes [m].
 * surface_pressure_pa: atmospheric absolute pressure at the water surface [Pa].
 * water_density_kg_m3: constant water density [kg/m^3].
 * gravity_m_s2: gravitational acceleration magnitude [m/s^2].
 */
inline gtsam::Expression<double> predict_pressure(
    const gtsam::Pose3_& IMU_pose,
    const gtsam::Point3& pressure_port_offset_IMU_m,
    const double surface_pressure_pa,
    const double water_density_kg_m3,
    const double gravity_m_s2) {
    const auto p_surface_pa = gtsam::Double_(surface_pressure_pa);
    const auto rho_kg_m3 = water_density_kg_m3;
    const auto g_m_s2 = gravity_m_s2;
    const auto down_WORLD = gtsam::Vector3_(gtsam::Vector3::UnitZ());
    const auto port_offset_IMU_m = gtsam::Point3_(pressure_port_offset_IMU_m);

    const auto port_position_WORLD_m =
        gtsam::transformFrom(IMU_pose, port_offset_IMU_m);
    const auto port_depth_m = gtsam::dot(down_WORLD, port_position_WORLD_m);

    return p_surface_pa + (rho_kg_m3 * g_m_s2) * port_depth_m;
}

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__PRESSURE_HPP_

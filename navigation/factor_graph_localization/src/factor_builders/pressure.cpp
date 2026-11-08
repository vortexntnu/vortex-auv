#include "factor_graph_localization/factor_builders/pressure.hpp"

#include <gtsam/linear/NoiseModel.h>

#include "factor_graph_localization/expressions/pressure.hpp"

namespace vortex::navigation {

gtsam::ExpressionFactor<double> build_pressure_factor(
    const gtsam::Key IMU_pose_key,
    const PressureMeasurement& pressure_measurement,
    const PressureGeometry& pressure_geometry,
    const PressureModel& pressure_model) {
    const gtsam::Pose3_ IMU_pose(IMU_pose_key);

    const auto h = predict_pressure(
        IMU_pose, pressure_geometry.IMU_PORT_OFFSET,
        pressure_model.surface_pressure_pa, pressure_model.water_density_kg_m3,
        pressure_model.gravity_mps2);

    const auto pressure_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(1, pressure_measurement.sigma_pa);

    return gtsam::ExpressionFactor<double>(
        pressure_noise_model, pressure_measurement.pressure_pa, h);
}

}  // namespace vortex::navigation

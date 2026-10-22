#include "factor_graph_localization/factor_builders/dvl_beam.hpp"

#include <gtsam/linear/NoiseModel.h>

#include "factor_graph_localization/expressions/dvl_beam.hpp"

namespace vortex::navigation {

gtsam::ExpressionFactor<double> build_dvl_beam_factor(
    const gtsam::Key IMU_origin_vel,
    const DvlBeamMeasurement& beam_measurement,
    const DvlBeamGeometry& beam_geometry,
    const gtsam::Vector3& angular_vel) {
    const auto& measured_beam_vel_mps = beam_measurement.vel_mps;
    const auto& sigma_b = beam_measurement.sigma_mps;

    // The optimizer needs to vary this velocity to minimize total graph error.
    // Hence, reference its graph key instead of supplying a fixed velocity
    // value.
    const gtsam::Vector3_ v_IMU(IMU_origin_vel);

    const auto h =
        predict_dvl_beam_vel(v_IMU, beam_geometry.IMU_P_DVL, angular_vel,
                             beam_geometry.beam_direction);

    const auto beam_vel_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_b);

    return gtsam::ExpressionFactor<double>(beam_vel_noise_model,
                                           measured_beam_vel_mps, h);
}

}  // namespace vortex::navigation

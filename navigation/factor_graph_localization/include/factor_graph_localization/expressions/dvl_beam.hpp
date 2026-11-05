#ifndef FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_
#define FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/expressions.h>

namespace vortex::navigation {

using ImuBias_ = gtsam::Expression<gtsam::imuBias::ConstantBias>;

/**
 * @brief DVL beam prediction [m/s] from the IMU graph state.
 *
 * IMU is the navigation sensing frame; WORLD is fixed at initialization.
 * WORLD_P_IMU: graph pose mapping IMU into WORLD.
 * vel_WORLD: IMU-origin velocity in WORLD axes [m/s].
 * IMU_P_DVL: fixed DVL -> IMU mounting transform. Its translation points
 * from the IMU origin to the DVL virtual origin, in IMU axes [m].
 * measured_angular_vel_IMU: gyro measurement [rad/s].
 * beam_direction: outward beam direction in DVL axes.
 */
inline gtsam::Expression<double> predict_dvl_beam_vel(
    const gtsam::Pose3_& WORLD_P_IMU,
    const gtsam::Vector3_& vel_WORLD,
    const ImuBias_& estimated_accel_gyro_bias_IMU,
    const gtsam::Pose3& IMU_P_DVL,
    const gtsam::Vector3& measured_angular_vel_IMU,
    const gtsam::Unit3& beam_direction) {
    const auto& IMU_R_DVL = IMU_P_DVL.rotation();
    const auto& r = IMU_P_DVL.translation();

    // Share the IMU factor's WORLD velocity key while allowing optimized
    // attitude changes to update the velocity expressed in IMU axes.
    const auto vel_IMU =
        gtsam::unrotate(gtsam::rotation(WORLD_P_IMU), vel_WORLD);

    const gtsam::Vector3_ angular_vel_IMU(
        estimated_accel_gyro_bias_IMU,
        &gtsam::imuBias::ConstantBias::correctGyroscope,
        gtsam::Vector3_(measured_angular_vel_IMU));
    const auto vel_lever_arm =
        gtsam::cross(angular_vel_IMU, gtsam::Vector3_(r));

    const auto vel_DVL_in_IMU = vel_IMU + vel_lever_arm;

    const auto vel_DVL_in_DVL =
        gtsam::unrotate(gtsam::Rot3_(IMU_R_DVL), vel_DVL_in_IMU);

    return gtsam::dot(gtsam::Vector3_(beam_direction.unitVector()),
                      vel_DVL_in_DVL);
}

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_

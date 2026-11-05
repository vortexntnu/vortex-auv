#ifndef FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_
#define FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/slam/expressions.h>

namespace vortex::navigation {

/**
 * @brief DVL beam prediction [m/s] from the IMU graph pose and velocity.
 *
 * IMU is the navigation sensing frame; STARTUP is GTSAM's fixed reference.
 * STARTUP_P_IMU: graph pose mapping IMU into STARTUP.
 * vel_STARTUP: graph velocity of the IMU origin relative to the stationary
 * bottom, expressed in STARTUP axes [m/s].
 * IMU_P_DVL: fixed DVL -> IMU mounting transform. Its translation points
 * from the IMU origin to the DVL virtual origin, in IMU axes [m].
 * angular_vel_IMU: bias-corrected gyro [rad/s] at the beam timestamp.
 * beam_direction: outward beam direction in DVL axes, from sensors.yaml.
 * Returns the predicted beam velocity in its measurement space. Inputs and
 * mounting calibration are caller preconditions; no validation is performed.
 */
inline gtsam::Expression<double> predict_dvl_beam_vel(
    const gtsam::Pose3_& STARTUP_P_IMU,
    const gtsam::Vector3_& vel_STARTUP,
    const gtsam::Pose3& IMU_P_DVL,
    const gtsam::Vector3& angular_vel_IMU,
    const gtsam::Unit3& beam_direction) {
    const auto& IMU_R_DVL = IMU_P_DVL.rotation();
    const auto& r = IMU_P_DVL.translation();

    // Share the IMU factor's STARTUP velocity key while allowing optimized
    // attitude changes to update the velocity expressed in IMU axes.
    const auto vel_IMU =
        gtsam::unrotate(gtsam::rotation(STARTUP_P_IMU), vel_STARTUP);

    const gtsam::Vector3 vel_lever_arm = angular_vel_IMU.cross(r);

    const auto vel_DVL_in_IMU = vel_IMU + gtsam::Vector3_(vel_lever_arm);

    const auto vel_DVL_in_DVL =
        gtsam::unrotate(gtsam::Rot3_(IMU_R_DVL), vel_DVL_in_IMU);

    return gtsam::dot(gtsam::Vector3_(beam_direction.unitVector()),
                      vel_DVL_in_DVL);
}

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_

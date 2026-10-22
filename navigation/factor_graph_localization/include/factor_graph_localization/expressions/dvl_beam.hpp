#ifndef FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_
#define FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/slam/expressions.h>

namespace vortex::navigation {

/**
 * @brief DVL beam prediction h(v_IMU) [m/s], assuming a stationary bottom.
 *
 * IMU = STIM300 sensing frame; DVL = Nucleus DVL virtual reference frame.
 * v_IMU: IMU-origin velocity relative to bottom, in IMU axes [m/s].
 * omega: bias-corrected angular velocity in IMU axes [rad/s].
 * r: IMU -> DVL origin offset in IMU axes [m].
 * IMU_R_DVL: DVL -> IMU rotation.
 * beam_direction: outward beam direction in DVL axes, from sensors.yaml.
 */
inline gtsam::Expression<double> predict_dvl_beam_vel(
    const gtsam::Vector3_& v_IMU,
    const gtsam::Pose3& IMU_P_DVL,
    const gtsam::Vector3& omega,
    const gtsam::Unit3& beam_direction) {
    const auto& IMU_R_DVL = IMU_P_DVL.rotation();
    const auto& r = IMU_P_DVL.translation();

    const gtsam::Vector3 v_lever_arm = omega.cross(r);
    
    const gtsam::Vector3_ v_DVL_in_IMU =
        v_IMU + gtsam::Vector3_(v_lever_arm);
    
    const auto v_DVL_in_DVL =
        gtsam::unrotate(gtsam::Rot3_(IMU_R_DVL), v_DVL_in_IMU);

    return gtsam::dot(gtsam::Vector3_(beam_direction.unitVector()), v_DVL_in_DVL);
}

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__EXPRESSIONS__DVL_BEAM_HPP_

#ifndef FACTOR_GRAPH_LOCALIZATION__ESTIMATION__INITIALIZATION_HPP_
#define FACTOR_GRAPH_LOCALIZATION__ESTIMATION__INITIALIZATION_HPP_

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <optional>

namespace vortex::navigation {

struct StartupMagnetometerReference {
    gtsam::Rot3 WORLD_R_IMU_start;
    gtsam::Vector3 field_WORLD_tesla;
};

struct InitialState {
    double timestamp_s;
    gtsam::Pose3 WORLD_P_IMU;
    gtsam::Vector3 IMU_origin_vel_IMU_mps;
    gtsam::imuBias::ConstantBias bias_IMU;
    std::optional<gtsam::Vector3> field_WORLD_tesla;
};

struct InitialPriorCovariances {
    gtsam::Matrix6 pose_covariance_rot_pos;
    gtsam::Matrix3 velocity_covariance_WORLD_m2ps2;
    gtsam::Matrix6 bias_covariance_accel_gyro_IMU;
};

struct InitialGraph {
    double timestamp_s;
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    std::optional<gtsam::Vector3> field_WORLD_tesla;
};

InitialState make_initial_state_from_startup(
    const double timestamp_s,
    const StartupMagnetometerReference& startup_reference,
    const gtsam::Point3& IMU_origin_pos_WORLD_m,
    const gtsam::Vector3& IMU_origin_vel_IMU_mps,
    const gtsam::imuBias::ConstantBias& bias_IMU);

InitialGraph make_initial_graph(
    const gtsam::Key IMU_pose_key,
    const gtsam::Key IMU_origin_vel_WORLD_key,
    const gtsam::Key IMU_bias_key,
    const InitialState& initial_state,
    const InitialPriorCovariances& prior_covariances);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__ESTIMATION__INITIALIZATION_HPP_

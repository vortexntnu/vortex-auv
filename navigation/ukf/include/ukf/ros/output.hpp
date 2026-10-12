/**
 * @file output.hpp
 * @brief Contains the mapping from the 15-dimensional state covariance to the
 * 6x6 pose and twist covariances of the ROS odometry message.
 */

#ifndef UKF__ROS__OUTPUT_HPP_
#define UKF__ROS__OUTPUT_HPP_

#include <vortex/utils/math.hpp>
#include "ukf/typedefs.hpp"

namespace ukf_output {

using Matrix6 = Eigen::Matrix<double, 6, 6>;

/**
 * @brief The pose covariance, position and rotation in the parent frame.
 * @param state The state the covariance is linearised about.
 * @param P The state covariance.
 * @return The 6x6 pose covariance.
 */
inline Matrix6 pose_covariance(const State& state, const Eigen::Matrix15d& P) {
    Eigen::Matrix<double, 6, 15> J = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3, 3>(0, StateIndex::position).setIdentity();
    // ROS pose rotation errors are small fixed-axis rotations in the parent
    // frame
    J.block<3, 3>(3, StateIndex::attitude) = state.quat.toRotationMatrix();
    return J * P * J.transpose();
}

/**
 * @brief The twist covariance, body-frame velocity and angular velocity.
 * @param state The state the covariance is linearised about.
 * @param P The state covariance.
 * @param gyro_covariance The covariance of the current gyro sample, which is
 * assumed independent of the state error.
 * @return The 6x6 twist covariance.
 */
inline Matrix6 twist_covariance(const State& state,
                                const Eigen::Matrix15d& P,
                                const Eigen::Matrix3d& gyro_covariance) {
    Eigen::Matrix<double, 6, 15> J = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3, 3>(0, StateIndex::velocity) =
        state.quat.toRotationMatrix().transpose();
    J.block<3, 3>(0, StateIndex::attitude) =
        vortex::utils::math::get_skew_symmetric_matrix(state.quat.conjugate() *
                                                       state.vel);
    J.block<3, 3>(3, StateIndex::gyro_bias) = -Eigen::Matrix3d::Identity();
    Matrix6 covariance = J * P * J.transpose();
    covariance.bottomRightCorner<3, 3>() += gyro_covariance;
    return covariance;
}

}  // namespace ukf_output

#endif  // UKF__ROS__OUTPUT_HPP_

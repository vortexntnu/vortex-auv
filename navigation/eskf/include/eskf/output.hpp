#pragma once
#include "eskf/lie.hpp"
#include "eskf/typedefs.hpp"
namespace eskf_output {
using Matrix6 = Eigen::Matrix<double, 6, 6>;
inline Matrix6 pose_covariance(const NominalState& state,
                               const Eigen::Matrix15d& P) {
    Eigen::Matrix<double, 6, 15> J = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3, 3>(0, StateIndex::position).setIdentity();
    // ROS pose rotation errors are small fixed-axis rotations in the parent
    // frame.
    J.block<3, 3>(3, StateIndex::attitude) = state.quat.toRotationMatrix();
    return J * P * J.transpose();
}
inline Matrix6 twist_covariance(const NominalState& state,
                                const Eigen::Matrix15d& P,
                                const Eigen::Matrix3d& gyro_covariance) {
    Eigen::Matrix<double, 6, 15> J = Eigen::Matrix<double, 6, 15>::Zero();
    J.block<3, 3>(0, StateIndex::velocity) =
        state.quat.toRotationMatrix().transpose();
    J.block<3, 3>(0, StateIndex::attitude) =
        eskf_math::skew(state.quat.conjugate() * state.vel);
    J.block<3, 3>(3, StateIndex::gyro_bias) = -Eigen::Matrix3d::Identity();
    Matrix6 covariance = J * P * J.transpose();
    // Approximation: current gyro sample noise is independent of the state
    // error.
    covariance.bottomRightCorner<3, 3>() += gyro_covariance;
    return covariance;
}
}  // namespace eskf_output

#include "ukf/models/strapdown_ins.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <vortex/utils/math.hpp>
#include "ukf/lie/SO3.hpp"
#include "ukf/typedefs.hpp"

using vortex::utils::math::get_skew_symmetric_matrix;

// clang-format off
StrapdownINS3D::StrapdownINS3D(const INS3DConfig& config) : config_(config) {}

State StrapdownINS3D::f(const State& state, const ImuInput& input) const {
    // biases follow random walks, so they are not changed by the prediction
    State next = state;
    const Eigen::Vector3d acceleration = state.quat * (input.accel - state.accel_bias) + config_.gravity;

    next.pos = state.pos + state.vel * input.dt + 0.5 * acceleration * input.dt * input.dt;
    next.vel = state.vel + acceleration * input.dt;
    next.quat = (state.quat * SO3::Exp((input.gyro - state.gyro_bias) * input.dt)).normalized();

    return next;
}

Eigen::Matrix15d StrapdownINS3D::Q(const State& state,
                                   const ImuInput& input) const {
    const Eigen::Matrix3d R = state.quat.toRotationMatrix();

    // continuous time error state dynamics, linearised about the state
    Eigen::Matrix15d A = Eigen::Matrix15d::Zero();
    A.block<3, 3>(StateIndex::position, StateIndex::velocity).setIdentity();
    A.block<3, 3>(StateIndex::velocity, StateIndex::attitude) = -R * get_skew_symmetric_matrix(input.accel - state.accel_bias);
    A.block<3, 3>(StateIndex::velocity, StateIndex::accel_bias) = -R;
    A.block<3, 3>(StateIndex::attitude, StateIndex::attitude) = -get_skew_symmetric_matrix(input.gyro - state.gyro_bias);
    A.block<3, 3>(StateIndex::attitude, StateIndex::gyro_bias) = -Eigen::Matrix3d::Identity();

    // how the continuous time noise enters the error state
    Eigen::Matrix15x12d G = Eigen::Matrix15x12d::Zero();
    G.block<3, 3>(StateIndex::velocity, NoiseIndex::acceleration) = -R;
    G.block<3, 3>(StateIndex::attitude, NoiseIndex::gyro) = -Eigen::Matrix3d::Identity();
    G.block<3, 3>(StateIndex::gyro_bias, NoiseIndex::gyro_bias).setIdentity();
    G.block<3, 3>(StateIndex::accel_bias, NoiseIndex::accel_bias).setIdentity();

    // Van Loan, the exponential of
    //   [ -A   G*Q*G^T ]
    //   [  0      A^T  ] * dt
    Eigen::Matrix30d van_loan = Eigen::Matrix30d::Zero();
    van_loan.topLeftCorner<15, 15>() = -A;
    van_loan.topRightCorner<15, 15>() = G * config_.noise_psd * G.transpose();
    van_loan.bottomRightCorner<15, 15>() = A.transpose();

    const Eigen::Matrix30d exponential = (van_loan * input.dt).exp();
    const Eigen::Matrix15d transition = exponential.bottomRightCorner<15, 15>().transpose();
    const Eigen::Matrix15d Q_discrete = transition * exponential.topRightCorner<15, 15>();

    return 0.5 * (Q_discrete + Q_discrete.transpose());
}

State StrapdownINS3D::composition_plus(const State& state,
                                       const Eigen::Vector15d& delta) const {
    State perturbed_state = state;
    perturbed_state.pos += delta.segment<3>(StateIndex::position);
    perturbed_state.vel += delta.segment<3>(StateIndex::velocity);
    perturbed_state.quat = (state.quat * SO3::Exp(delta.segment<3>(StateIndex::attitude))).normalized();
    perturbed_state.gyro_bias += delta.segment<3>(StateIndex::gyro_bias);
    perturbed_state.accel_bias += delta.segment<3>(StateIndex::accel_bias);

    return perturbed_state;
}

Eigen::Vector15d StrapdownINS3D::composition_minus(const State& state_a,
                                                   const State& state_b) const {
    Eigen::Vector15d delta;
    delta.segment<3>(StateIndex::position) = state_a.pos - state_b.pos;
    delta.segment<3>(StateIndex::velocity) = state_a.vel - state_b.vel;
    delta.segment<3>(StateIndex::attitude) = SO3::Log(state_b.quat.conjugate() * state_a.quat);
    delta.segment<3>(StateIndex::gyro_bias) = state_a.gyro_bias - state_b.gyro_bias;
    delta.segment<3>(StateIndex::accel_bias) = state_a.accel_bias - state_b.accel_bias;

    return delta;
}

//clang-format on

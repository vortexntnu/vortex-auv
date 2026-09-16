#include "eskf/eskf.hpp"
#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>
#include "eskf/lie.hpp"
using eskf_math::skew;

ESKF::ESKF(const EskfParams& params, const NominalState& initial)
    : params_(params), current_nom_state_(initial) {
    const auto covariance_valid = [](const auto& matrix) {
        return matrix.allFinite() &&
               matrix.isApprox(matrix.transpose(), 1e-10) &&
               matrix.ldlt().isPositive();
    };
    if (!covariance_valid(params.P) || !covariance_valid(params.Q) ||
        !params.g_.allFinite() || !initial.as_vector().allFinite() ||
        initial.quat.norm() < 1e-12 || !std::isfinite(params.max_imu_dt) ||
        params.max_imu_dt <= 0 || !std::isfinite(params.dvl_nis_threshold) ||
        params.dvl_nis_threshold <= 0 ||
        !std::isfinite(params.depth_nis_threshold) ||
        params.depth_nis_threshold <= 0)
        throw std::invalid_argument(
            "Invalid ESKF state, covariance or configuration");
    current_nom_state_.quat.normalize();
    current_error_state_.covariance = params.P;
}
Eigen::Vector3d calculate_h(const NominalState& state) {
    return state.quat.conjugate() * state.vel;
}
Eigen::Matrix3x15d calculate_h_jacobian(const NominalState& state) {
    Eigen::Matrix3x15d H = Eigen::Matrix3x15d::Zero();
    H.block<3, 3>(0, StateIndex::velocity) =
        state.quat.toRotationMatrix().transpose();
    H.block<3, 3>(0, StateIndex::attitude) = skew(calculate_h(state));
    return H;
}
void ESKF::nominal_state_discrete(const ImuMeasurement& imu, double dt) {
    const Eigen::Vector3d acceleration =
        current_nom_state_.quat * (imu.accel - current_nom_state_.accel_bias) +
        params_.g_;
    current_nom_state_.pos +=
        current_nom_state_.vel * dt + 0.5 * acceleration * dt * dt;
    current_nom_state_.vel += acceleration * dt;
    current_nom_state_.quat =
        (current_nom_state_.quat *
         eskf_math::exp((imu.gyro - current_nom_state_.gyro_bias) * dt))
            .normalized();
    // Biases follow random walks: no deterministic decay of mean or error.
}
void ESKF::error_state_prediction(const ImuMeasurement& imu, double dt) {
    const Eigen::Matrix3d R = current_nom_state_.quat.toRotationMatrix();
    Eigen::Matrix15d A = Eigen::Matrix15d::Zero();
    A.block<3, 3>(StateIndex::position, StateIndex::velocity).setIdentity();
    A.block<3, 3>(StateIndex::velocity, StateIndex::attitude) =
        -R * skew(imu.accel - current_nom_state_.accel_bias);
    A.block<3, 3>(StateIndex::attitude, StateIndex::attitude) =
        -skew(imu.gyro - current_nom_state_.gyro_bias);
    A.block<3, 3>(StateIndex::velocity, StateIndex::accel_bias) = -R;
    A.block<3, 3>(StateIndex::attitude, StateIndex::gyro_bias) =
        -Eigen::Matrix3d::Identity();
    Eigen::Matrix15x12d G = Eigen::Matrix15x12d::Zero();
    G.block<3, 3>(StateIndex::velocity, NoiseIndex::acceleration) = -R;
    G.block<3, 3>(StateIndex::attitude, NoiseIndex::gyro) =
        -Eigen::Matrix3d::Identity();
    G.block<3, 3>(StateIndex::gyro_bias, NoiseIndex::gyro_bias).setIdentity();
    G.block<3, 3>(StateIndex::accel_bias, NoiseIndex::accel_bias).setIdentity();
    Eigen::Matrix30d van_loan = Eigen::Matrix30d::Zero();
    van_loan.topLeftCorner<15, 15>() = -A;
    van_loan.topRightCorner<15, 15>() = G * params_.Q * G.transpose();
    van_loan.bottomRightCorner<15, 15>() = A.transpose();
    const Eigen::Matrix30d exponential = (van_loan * dt).exp();
    const Eigen::Matrix15d transition =
        exponential.bottomRightCorner<15, 15>().transpose();
    const Eigen::Matrix15d next =
        transition * current_error_state_.covariance * transition.transpose() +
        transition * exponential.topRightCorner<15, 15>();
    current_error_state_.covariance = 0.5 * (next + next.transpose());
}
void ESKF::injection_and_reset() {
    current_nom_state_.pos += current_error_state_.pos;
    current_nom_state_.vel += current_error_state_.vel;
    current_nom_state_.quat = (current_nom_state_.quat *
                               eskf_math::exp(current_error_state_.rotation))
                                  .normalized();
    current_nom_state_.gyro_bias += current_error_state_.gyro_bias;
    current_nom_state_.accel_bias += current_error_state_.accel_bias;
    Eigen::Matrix15d reset = Eigen::Matrix15d::Identity();
    reset.block<3, 3>(StateIndex::attitude, StateIndex::attitude) =
        eskf_math::right_jacobian(current_error_state_.rotation);
    const Eigen::Matrix15d transported =
        reset * current_error_state_.covariance * reset.transpose();
    current_error_state_.covariance =
        0.5 * (transported + transported.transpose());
    current_error_state_.set_from_vector(Eigen::Vector15d::Zero());
}
bool ESKF::imu_update(const ImuMeasurement& imu, double dt) {
    if (!std::isfinite(dt) || dt <= 0 || dt > params_.max_imu_dt ||
        !imu.accel.allFinite() || !imu.gyro.allFinite())
        return false;
    const auto before_nominal = current_nom_state_;
    const auto before_error = current_error_state_;
    // Linearize at the beginning of the same interval as the nominal
    // integrator.
    error_state_prediction(imu, dt);
    nominal_state_discrete(imu, dt);
    if (!current_nom_state_.as_vector().allFinite() ||
        !current_error_state_.covariance.allFinite()) {
        current_nom_state_ = before_nominal;
        current_error_state_ = before_error;
        return false;
    }
    return true;
}
bool ESKF::dvl_update(const SensorDVL& sensor) {
    return measurement_update(sensor, params_.dvl_nis_threshold, nis_dvl_);
}
bool ESKF::depth_update(const SensorDepth& sensor) {
    return measurement_update(sensor, params_.depth_nis_threshold, nis_depth_);
}
Eigen::Vector3d SensorDVL::innovation(const NominalState& state) const {
    return measurement - calculate_h(state);
}
Eigen::Matrix3x15d SensorDVL::jacobian(const NominalState& state) const {
    return calculate_h_jacobian(state);
}
Eigen::Matrix<double, 1, 1> SensorDepth::innovation(
    const NominalState& state) const {
    return Eigen::Matrix<double, 1, 1>::Constant(measurement - state.pos.z() -
                                                 (state.quat * lever_arm).z());
}
Eigen::Matrix<double, 1, 15> SensorDepth::jacobian(
    const NominalState& state) const {
    Eigen::Matrix<double, 1, 15> H = Eigen::Matrix<double, 1, 15>::Zero();
    H(0, StateIndex::position + 2) = 1;
    H.block<1, 3>(0, StateIndex::attitude) =
        -(state.quat.toRotationMatrix() * skew(lever_arm)).row(2);
    return H;
}

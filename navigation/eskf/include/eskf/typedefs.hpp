#ifndef ESKF__TYPEDEFS_HPP_
#define ESKF__TYPEDEFS_HPP_
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <concepts>
#include <stdexcept>
#include <vector>

namespace Eigen {
using Matrix12d = Matrix<double, 12, 12>;
using Matrix15d = Matrix<double, 15, 15>;
using Matrix15x12d = Matrix<double, 15, 12>;
using Matrix30d = Matrix<double, 30, 30>;
using Vector12d = Matrix<double, 12, 1>;
using Vector15d = Matrix<double, 15, 1>;
using Vector16d = Matrix<double, 16, 1>;
using Matrix3x15d = Matrix<double, 3, 15>;
}  // namespace Eigen
// Right-local attitude error; position and velocity errors are
// navigation-frame.
struct StateIndex {
    static constexpr int position = 0, velocity = 3, attitude = 6;
    static constexpr int gyro_bias = 9, accel_bias = 12;
};
struct NoiseIndex {
    static constexpr int acceleration = 0, gyro = 3, gyro_bias = 6,
                         accel_bias = 9;
};
template <int N>
Eigen::Matrix<double, N, N> createDiagonalMatrix(
    const std::vector<double>& diag) {
    if (diag.size() != N)
        throw std::invalid_argument("Invalid covariance diagonal size");
    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(diag.data())
        .asDiagonal();
}
struct NominalState {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector16d as_vector() const {
        Eigen::Vector16d x;
        x << pos, vel, quat.w(), quat.x(), quat.y(), quat.z(), gyro_bias,
            accel_bias;
        return x;
    }
};
struct ErrorState {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Matrix15d covariance = Eigen::Matrix15d::Zero();
    Eigen::Vector15d as_vector() const {
        Eigen::Vector15d x;
        x << pos, vel, rotation, gyro_bias, accel_bias;
        return x;
    }
    void set_from_vector(const Eigen::Vector15d& x) {
        pos = x.segment<3>(StateIndex::position);
        vel = x.segment<3>(StateIndex::velocity);
        rotation = x.segment<3>(StateIndex::attitude);
        gyro_bias = x.segment<3>(StateIndex::gyro_bias);
        accel_bias = x.segment<3>(StateIndex::accel_bias);
    }
};
struct EskfParams {
    // Continuous-time noise PSD: acceleration, gyro, gyro-bias RW, accel-bias
    // RW.
    Eigen::Matrix12d Q = Eigen::Matrix12d::Zero();
    Eigen::Matrix15d P = Eigen::Matrix15d::Identity();
    Eigen::Vector3d g_{0.0, 0.0, 9.82841};
    double max_imu_dt = 0.1;
    double dvl_nis_threshold = 16.266;    // chi-square(3), 99.9%.
    double depth_nis_threshold = 10.828;  // chi-square(1), 99.9%.
};
struct ImuMeasurement {
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};
struct SensorDVL {
    static constexpr int dimension = 3;
    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    Eigen::Matrix3d measurement_noise = Eigen::Matrix3d::Identity();
    Eigen::Vector3d innovation(const NominalState& state) const;
    Eigen::Matrix3x15d jacobian(const NominalState& state) const;
    Eigen::Matrix3d noise_covariance() const { return measurement_noise; }
};
struct SensorDepth {
    static constexpr int dimension = 1;
    double measurement = 0;
    double measurement_noise = 1;
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 1, 1> innovation(const NominalState& state) const;
    Eigen::Matrix<double, 1, 15> jacobian(const NominalState& state) const;
    Eigen::Matrix<double, 1, 1> noise_covariance() const {
        return Eigen::Matrix<double, 1, 1>::Constant(measurement_noise);
    }
};
template <typename T>
concept SensorModelConcept =
    requires(const T& sensor, const NominalState& state) {
        { T::dimension } -> std::convertible_to<int>;
        sensor.innovation(state);
        sensor.jacobian(state);
        sensor.noise_covariance();
    };
#endif

/**
 * @file typedefs.hpp
 * @brief Contains the typedef and structs for the eskf.
 */
#ifndef ESKF_TYPEDEFS_H
#define ESKF_TYPEDEFS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <concepts>

namespace Eigen {
typedef Eigen::Matrix<double, 19, 1> Vector19d;
typedef Eigen::Matrix<double, 18, 1> Vector18d;
typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 19, 19> Matrix19d;
typedef Eigen::Matrix<double, 18, 12> Matrix18x12d;
typedef Eigen::Matrix<double, 4, 3> Matrix4x3d;
typedef Eigen::Matrix<double, 3, 19> Matrix3x19d;
typedef Eigen::Matrix<double, 3, 18> Matrix3x18d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1d;
typedef Eigen::Matrix<double, 19, 18> Matrix19x18d;
typedef Eigen::Matrix<double, 18, 3> Matrix18x3d;
typedef Eigen::Matrix<double, 36, 36> Matrix36d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 16, 1> Vector16d;
typedef Eigen::Matrix<double, 15, 12> Matrix15x12d;
typedef Eigen::Matrix<double, 16, 15> Matrix16x15d;
typedef Eigen::Matrix<double, 3, 15> Matrix3x15d;
typedef Eigen::Matrix<double, 3, 16> Matrix3x16d;
typedef Eigen::Matrix<double, 30, 30> Matrix30d;
}  // namespace Eigen

template <int N>
Eigen::Matrix<double, N, N> createDiagonalMatrix(
    const std::vector<double>& diag) {
    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(diag.data())
        .asDiagonal();
}
struct StateQuat {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();

    StateQuat() = default;

    Eigen::Vector16d as_vector() const {
        Eigen::Vector16d vec{};
        vec << pos, vel, quat.w(), quat.x(), quat.y(), quat.z(), gyro_bias,
            accel_bias;
        return vec;
    }

    StateQuat operator-(const StateQuat& other) const {
        StateQuat diff{};
        diff.pos = pos - other.pos;
        diff.vel = vel - other.vel;
        diff.quat = quat * other.quat.inverse();
        diff.gyro_bias = gyro_bias - other.gyro_bias;
        diff.accel_bias = accel_bias - other.accel_bias;
        return diff;
    }
};

struct StateEuler {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();

    Eigen::Matrix15d covariance = Eigen::Matrix15d::Zero();

    Eigen::Vector15d as_vector() const {
        Eigen::Vector15d vec{};
        vec << pos, vel, euler, gyro_bias, accel_bias;
        return vec;
    }

    void set_from_vector(const Eigen::Vector15d& vec) {
        pos = vec.block<3, 1>(0, 0);
        vel = vec.block<3, 1>(3, 0);
        euler = vec.block<3, 1>(6, 0);
        gyro_bias = vec.block<3, 1>(9, 0);
        accel_bias = vec.block<3, 1>(12, 0);
    }
};

struct EskfParams {
    Eigen::Matrix12d Q = Eigen::Matrix12d::Zero();
    Eigen::Matrix15d P = Eigen::Matrix15d::Zero();
};
struct ImuMeasurement {
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct DvlMeasurement {
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
};

template<typename T>
concept SensorModelConcept = requires(const T &meas, const StateQuat &state)
{
    { meas.innovation(state) } -> std::convertible_to<Eigen::VectorXd>;
    { meas.jacobian(state) } -> std::convertible_to<Eigen::MatrixXd>;
    { meas.noise_covariance() } -> std::convertible_to<Eigen::MatrixXd>;
};

struct SensorDVL {
    Eigen::Vector3d measurement;
    Eigen::Matrix3d measurement_noise;
    Eigen::VectorXd innovation(const StateQuat &state) const;
    Eigen::MatrixXd jacobian(const StateQuat &state) const;
    Eigen::MatrixXd noise_covariance() const;
};


#endif  // ESKF_TYPEDEFS_H

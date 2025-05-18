/**
 * @file typedefs.hpp
 * @brief Contains the typedef and structs for the eskf.
 */
#ifndef ESKF_TYPEDEFS_H
#define ESKF_TYPEDEFS_H

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

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
typedef Eigen::Matrix<double, 15, 1> Vector15d;
}  // namespace Eigen

template <int N>
Eigen::Matrix<double, N, N> createDiagonalMatrix(
    const std::vector<double>& diag) {
    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(diag.data())
        .asDiagonal();
}

struct state_quat {
    Eigen::Vector3d pos = Eigen::Vector3d(0,0,0.125);
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    state_quat() { gravity << 0, 0, 9.81; }

    Eigen::Vector19d as_vector() const {
        Eigen::Vector19d vec;
        vec << pos, vel, quat.w(), quat.x(), quat.y(), quat.z(), gyro_bias,
            accel_bias, gravity;
        return vec;
    }

    Eigen::Vector18d nees_error(const state_quat& other) const {
        Eigen::Vector18d vec;
        Eigen::Vector3d euler_diff;

        euler_diff = (quat * other.quat.inverse())
                         .toRotationMatrix()
                         .eulerAngles(0, 1, 2) + Eigen::Vector3d(-M_PI, M_PI, -M_PI);

        vec << pos - other.pos, vel - other.vel, euler_diff,
            gyro_bias - other.gyro_bias, accel_bias - other.accel_bias,
            gravity - other.gravity;
        return vec;
    }

    state_quat operator-(const state_quat& other) const {
        state_quat diff;
        diff.pos = pos - other.pos;
        diff.vel = vel - other.vel;
        diff.quat = quat * other.quat.inverse();
        diff.gyro_bias = gyro_bias - other.gyro_bias;
        diff.accel_bias = accel_bias - other.accel_bias;
        diff.gravity = gravity - other.gravity;
        return diff;
    }
};

struct state_euler {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    Eigen::Matrix18d covariance = Eigen::Matrix18d::Zero();

    Eigen::Vector18d as_vector() const {
        Eigen::Vector18d vec;
        vec << pos, vel, euler, gyro_bias, accel_bias, gravity;
        return vec;
    }

    void set_from_vector(const Eigen::Vector18d& vec) {
        pos = vec.block<3, 1>(0, 0);
        vel = vec.block<3, 1>(3, 0);
        euler = vec.block<3, 1>(6, 0);
        gyro_bias = vec.block<3, 1>(9, 0);
        accel_bias = vec.block<3, 1>(12, 0);
        gravity = vec.block<3, 1>(15, 0);
    }
};

struct imu_measurement {
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct dvl_measurement {
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
};

struct eskf_params {
    double temp = 0.0;
    Eigen::Matrix12d Q = Eigen::Matrix12d::Zero();
    double dt = 0.0;
};

#endif  // ESKF_TYPEDEFS_H

#ifndef UKF__TYPEDEFS_HPP_
#define UKF__TYPEDEFS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Eigen {
using Vector12d = Matrix<double, 12, 1>;
using Vector15d = Matrix<double, 15, 1>;
using Vector16d = Matrix<double, 16, 1>;
using Matrix12d = Matrix<double, 12, 12>;
using Matrix15d = Matrix<double, 15, 15>;
using Matrix15x12d = Matrix<double, 15, 12>;
}  // namespace Eigen

// tangent space of the state, attitude is a right-local rotation vector
struct StateIndex {
    static constexpr int position = 0, velocity = 3, attitude = 6;
    static constexpr int gyro_bias = 9, accel_bias = 12;
    static constexpr int size = 15;
};

struct NoiseIndex {
    static constexpr int acceleration = 0, gyro = 3, gyro_bias = 6,
                         accel_bias = 9;
    static constexpr int size = 12;
};

struct State {
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

struct ImuInput {
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    double dt = 0.0;
};

template <typename Point, int N>
struct Gaussian {
    Point mean;
    Eigen::Matrix<double, N, N> covariance;
};

using StateGaussian = Gaussian<State, StateIndex::size>;

template <int M>
using InnovationGaussian = Gaussian<Eigen::Matrix<double, M, 1>, M>;

// cross covariance between the state going in and the output coming out
template <typename Point, int N>
struct UnscentedTransformOutput {
    Gaussian<Point, N> transformed;
    Eigen::Matrix<double, StateIndex::size, N> cross_covariance;
};

template <int M>
struct UpdateOutput {
    StateGaussian posterior;
    InnovationGaussian<M> innovation;
};

struct TransformConfig {
    double alpha;
    double beta;
    double kappa;
};

#endif  // UKF__TYPEDEFS_HPP_

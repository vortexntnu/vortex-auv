/**
 * @file typedefs.hpp
 * @brief Contains the state, input and Gaussian types shared by the UKF.
 */

#ifndef UKF__TYPEDEFS_HPP_
#define UKF__TYPEDEFS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>
#include <vector>

/**
 * @brief Fixed-size Eigen aliases for the dimensions used by the filter.
 */
namespace Eigen {
using Vector12d = Matrix<double, 12, 1>;
using Vector15d = Matrix<double, 15, 1>;
using Vector16d = Matrix<double, 16, 1>;
using Matrix12d = Matrix<double, 12, 12>;
using Matrix15d = Matrix<double, 15, 15>;
using Matrix15x12d = Matrix<double, 15, 12>;
using Matrix30d = Matrix<double, 30, 30>;
}  // namespace Eigen

/**
 * @brief Builds a diagonal matrix from a vector of diagonal entries, such as a
 * ROS parameter.
 * @tparam N The size of the matrix.
 * @param diag The diagonal entries.
 * @return The N x N diagonal matrix.
 * @throws std::invalid_argument if diag does not have N entries.
 */
template <int N>
Eigen::Matrix<double, N, N> create_diagonal_matrix(
    const std::vector<double>& diag) {
    if (diag.size() != N) {
        throw std::invalid_argument("Invalid covariance diagonal size");
    }
    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(diag.data())
        .asDiagonal();
}

/**
 * @brief Index offsets into the 15-dimensional tangent space of the state.
 * The attitude is a right-local rotation vector.
 */
struct StateIndex {
    static constexpr int position = 0, velocity = 3, attitude = 6;
    static constexpr int gyro_bias = 9, accel_bias = 12;
    static constexpr int size = 15;
};

/**
 * @brief Index offsets into the 12-dimensional continuous-time process noise.
 */
struct NoiseIndex {
    static constexpr int acceleration = 0, gyro = 3, gyro_bias = 6,
                         accel_bias = 9;
    static constexpr int size = 12;
};

/**
 * @brief The navigation state, living on R^3 x R^3 x SO(3) x R^3 x R^3.
 */
struct State {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();

    /**
     * @brief Stacks the state into a vector, the quaternion stored as
     * (w, x, y, z).
     * @return The 16-dimensional state vector.
     */
    Eigen::Vector16d as_vector() const {
        Eigen::Vector16d x;
        x << pos, vel, quat.w(), quat.x(), quat.y(), quat.z(), gyro_bias,
            accel_bias;
        return x;
    }
};

/**
 * @brief The IMU measurement that drives the motion model.
 */
struct ImuInput {
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    double dt = 0.0;
};

/**
 * @brief A Gaussian distribution, a mean on a manifold together with the
 * covariance of its tangent space perturbation.
 * @tparam Point The manifold point type of the mean.
 * @tparam N The dimension of the tangent space.
 */
template <typename Point, int N>
struct Gaussian {
    Point mean;
    Eigen::Matrix<double, N, N> covariance;
};

/**
 * @brief Gaussian over the state.
 */
using StateGaussian = Gaussian<State, StateIndex::size>;

/**
 * @brief Gaussian over an M-dimensional innovation.
 */
template <int M>
using InnovationGaussian = Gaussian<Eigen::Matrix<double, M, 1>, M>;

/**
 * @brief The result of an unscented transform, the transformed Gaussian and
 * the cross covariance between the state going in and the output coming out.
 */
template <typename Point, int N>
struct UnscentedTransformOutput {
    Gaussian<Point, N> transformed;
    Eigen::Matrix<double, StateIndex::size, N> cross_covariance;
};

/**
 * @brief The result of a measurement update, the posterior state and the
 * innovation with its covariance S (used for NIS).
 */
template <int M>
struct UpdateOutput {
    StateGaussian posterior;
    InnovationGaussian<M> innovation;
};

/**
 * @brief Scaling parameters of the unscented transform.
 */
struct TransformConfig {
    double alpha;
    double beta;
    double kappa;
};

#endif  // UKF__TYPEDEFS_HPP_

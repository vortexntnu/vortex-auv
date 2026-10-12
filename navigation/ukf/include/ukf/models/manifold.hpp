/**
 * @file manifold.hpp
 * @brief Contains the Manifold and SensorModel concepts that the motion and
 * measurement models satisfy in order to be used by the unscented transform.
 */

#ifndef UKF__MODELS__MANIFOLD_HPP_
#define UKF__MODELS__MANIFOLD_HPP_

#include <Eigen/Dense>
#include <concepts>
#include "ukf/typedefs.hpp"

// clang-format off
/**
 * @brief The tangent space vector type of a manifold T.
 */
template <typename T>
using Tangent = Eigen::Matrix<double, T::dimension, 1>;

/**
 * @brief A space with points, a tangent space of size dimension and the
 * composition operators that respect the manifold. A type satisfying this
 * defines dimension, Point, composition_plus(point, delta) and
 * composition_minus(point_a, point_b).
 */
template <typename T>
concept Manifold = requires(const T& space,
                            const typename T::Point& point,
                            const Tangent<T>& delta) {
    { T::dimension } -> std::convertible_to<int>;
    { space.composition_plus(point, delta) } -> std::same_as<typename T::Point>;
    { space.composition_minus(point, point) } -> std::same_as<Tangent<T>>;
};

/**
 * @brief A Manifold that is also a measurement model, defining the
 * measurement function h(state, input) and the noise covariance
 * R(state, input).
 */
template <typename T>
concept SensorModel =
    Manifold<T> &&
    requires(const T& sensor, const State& state, const ImuInput& input) {
        { sensor.h(state, input) } -> std::same_as<typename T::Point>;
        { sensor.R(state, input) } -> std::same_as<Eigen::Matrix<double, T::dimension, T::dimension>>;
    };

#endif  // UKF__MODELS__MANIFOLD_HPP_
// clang-format on

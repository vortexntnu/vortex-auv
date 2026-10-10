#ifndef UKF__MODELS__MANIFOLD_HPP_
#define UKF__MODELS__MANIFOLD_HPP_

#include <Eigen/Dense>
#include <concepts>
#include "ukf/typedefs.hpp"

template <typename T>
using Tangent = Eigen::Matrix<double, T::dimension, 1>;

template <typename T>
concept Manifold = requires(const T& space,
                            const typename T::Point& point,
                            const Tangent<T>& delta) {
    { T::dimension } -> std::convertible_to<int>;
    { space.composition_plus(point, delta) } -> std::same_as<typename T::Point>;
    { space.composition_minus(point, point) } -> std::same_as<Tangent<T>>;
};  // NOLINT(readability/braces)

template <typename T>
concept SensorModel =
    Manifold<T> &&
    requires(const T& sensor, const State& state, const ImuInput& input) {
        { sensor.h(state, input) } -> std::same_as<typename T::Point>;
        {
            sensor.R(state, input)
        } -> std::same_as<Eigen::Matrix<double, T::dimension, T::dimension>>;
    };  // NOLINT(readability/braces)

#endif  // UKF__MODELS__MANIFOLD_HPP_

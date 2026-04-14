/**
 * @file thrust_allocator_utils.hpp
 * @brief This file contains utility functions for the thruster allocator
 * module.
 */

#ifndef THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_
#define THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <ranges>
#include <string>
#include <vector>
#include <vortex/utils/types.hpp>

using vortex::utils::types::Vector6d;

/**
 * @brief Check if the matrix has any NaN or INF elements.
 *
 * @tparam Derived The type of the matrix.
 * @param M The matrix to check.
 * @return true if the matrix has any NaN or INF elements, false otherwise.
 */
template <typename Derived>
inline bool is_invalid_matrix(const Eigen::MatrixBase<Derived>& M) {
    bool has_nan = !(M.array() == M.array()).all();
    bool has_inf = M.array().isInf().any();
    return has_nan || has_inf;
}

/**
 * @brief Saturates the values of a given Eigen vector between a minimum and
 * maximum value.
 *
 * @param vec The Eigen vector to be saturated.
 * @param min The minimum value to saturate the vector values to.
 * @param max The maximum value to saturate the vector values to.
 * @return True if all vector values are within the given range, false
 * otherwise.
 */
inline bool saturate_vector_values(Eigen::VectorXd& vec,
                                   double min,
                                   double max) {
    bool all_values_in_range = std::ranges::all_of(
        vec, [min, max](double val) { return val >= min && val <= max; });

    std::ranges::transform(vec, vec.begin(), [min, max](double val) {
        return std::min(std::max(val, min), max);
    });

    return all_values_in_range;
}

/**
 * @brief Converts a 1D array of doubles to a 2D Eigen matrix.
 *
 * @param matrix The 1D array of doubles to be converted.
 * @param rows The number of rows in the resulting Eigen matrix.
 * @param cols The number of columns in the resulting Eigen matrix.
 * @return The resulting Eigen matrix.
 */
inline Eigen ::MatrixXd double_array_to_eigen_matrix(
    const std::vector<double>& matrix,
    int rows,
    int cols) {
    return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic,
                                          Eigen::Dynamic, Eigen::RowMajor>>(
        matrix.data(), rows, cols);
}

inline Eigen::Vector3d double_array_to_eigen_vector3d(
    const std::vector<double>& vector) {
    // Ensure the input vector has exactly three elements
    if (vector.size() != 3) {
        throw std::invalid_argument(
            "Input vector must have exactly 3 elements");
    }

    // Map the vector to Eigen::Vector3d
    return Eigen::Map<const Eigen::Vector3d>(vector.data());
}

/**
 * @brief Clamps the wrench vector in a way that preserves scale between
 * elements, will spdlog if intervention was needed if in debug
 *
 * @param &tau reference to the desired wrench vector
 * @param tau_max the maximum allowed value of thrust
 * @return The normalized tau vector
 */
inline Eigen::VectorXd normalize_wrench_vector(const Eigen::VectorXd& tau,
                                               const Eigen::VectorXd& tau_max) {
    const Eigen::VectorXd normalized = tau.cwiseQuotient(tau_max);
    const double scale = std::max(1.0, normalized.cwiseAbs().maxCoeff());

#if !defined(NDEBUG)
    if (scale > 1.0) {
        spdlog::warn("Wrench scaled by factor {:.3f} for QP conditioning",
                     scale);
    }
#endif

    return tau / scale;
}

#endif  // THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_

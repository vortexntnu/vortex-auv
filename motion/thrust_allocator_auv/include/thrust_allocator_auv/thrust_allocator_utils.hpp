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

inline Eigen::MatrixXd calculate_thrust_configuration_matrix(
    const Eigen::MatrixXd& thruster_force_direction,
    const Eigen::MatrixXd& thruster_position,
    const Eigen::Vector3d& center_of_mass) {
    // Initialize thrust allocation matrix
    Eigen::MatrixXd thrust_configuration_matrix = Eigen::MatrixXd::Zero(6, 8);

    // Calculate thrust and moment contributions from each thruster
    for (int i = 0; i < thruster_position.cols(); i++) {
        Eigen::Vector3d pos =
            thruster_position.col(i);  // Thrust vector in body frame
        Eigen::Vector3d F =
            thruster_force_direction.col(i);  // Position vector in body frame

        // Calculate position vector relative to the center of mass
        pos -= center_of_mass;

        // Fill in the thrust allocation matrix
        thrust_configuration_matrix.block<3, 1>(0, i) = F;
        thrust_configuration_matrix.block<3, 1>(3, i) = pos.cross(F);
    }

    thrust_configuration_matrix = thrust_configuration_matrix.array();
    return thrust_configuration_matrix;
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
 * @brief Computes the maximum wrench that can be constructed in all DOFs
 *
 * @param &T reference to the thrust configuration matrix
 * @param u_min the minimum allowed value of thrust
 * @param u_max the maximum allowed value of thrust
 * @return The vector containing maximum value of thrust with the given thrust
 * configuration and limits
 */
inline Eigen::VectorXd compute_max_wrench(const Eigen::MatrixXd& T,
                                          const Eigen::VectorXd& u_min,
                                          const Eigen::VectorXd& u_max) {
    Eigen::VectorXd tau_max(T.rows());

    for (int i = 0; i < T.rows(); i++) {
        double w = 0.0;
        for (int j = 0; j < T.cols(); j++) {
            // For each DOF, greedily pick thruster contribution
            if (T(i, j) > 0)
                w += T(i, j) * u_max(j);
            else
                w += T(i, j) * u_min(j);
        }
        tau_max(i) = w;
    }

    return tau_max;
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

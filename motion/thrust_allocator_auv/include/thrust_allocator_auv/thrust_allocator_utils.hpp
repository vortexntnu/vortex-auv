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
#include "casadi/casadi.hpp"

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

inline casadi::DM eigen_to_dm(const Eigen::MatrixXd& M) {
    std::vector<double> data(M.data(), M.data() + M.size());
    casadi::Sparsity sp = casadi::Sparsity::dense(M.rows(), M.cols());
    return casadi::DM(sp, data, true);
}

inline casadi::DM eigen_to_dm(const Eigen::VectorXd& v) {
    std::vector<double> data(v.data(), v.data() + v.size());
    // This already becomes (N x 1)
    return casadi::DM(data);
}

inline Eigen::VectorXd dmToEigenVector(const casadi::DM& dm) {
    std::vector<double> data = static_cast<std::vector<double>>(dm);
    Eigen::VectorXd out(static_cast<int>(data.size()));
    for (int i = 0; i < out.size(); ++i)
        out[i] = data[static_cast<size_t>(i)];
    return out;
}

/**
 * @brief Clamps the wrench vector in a way that preserves scale between
 * elements, will spdlog if intervention was needed if in debug
 *
 * @param &tau reference to the desired wrench vector
 * @param tau_max the maximum allowed value of thrust
 * @return The normalized tau vector
 */
inline Eigen::VectorXd normalizeWrenchVector(const Eigen::VectorXd& tau,
                                             const double tau_max) {
    const double max_abs_component = tau.cwiseAbs().maxCoeff();
    const double scale = std::max(1.0, max_abs_component / tau_max);

#if !defined(NDEBUG)
    if (scale > 1.0) {
        spdlog::warn("Wrench scaled by factor {:.3f} for QP conditioning",
                     scale);
    }
#endif

    return tau / scale;
}

#endif  // THRUST_ALLOCATOR_AUV__THRUST_ALLOCATOR_UTILS_HPP_

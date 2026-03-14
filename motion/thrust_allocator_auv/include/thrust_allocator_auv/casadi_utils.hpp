
/**
 * @file casadi_utils.hpp
 * @brief This file contains utility functions for the thruster allocator
 * module that specifically use CasADi.
 */

#ifndef THRUST_ALLOCATOR_AUV__CASADI_UTILS_HPP_
#define THRUST_ALLOCATOR_AUV__CASADI_UTILS_HPP_

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/Eigen>
#include <vortex/utils/types.hpp>
#include "casadi/casadi.hpp"

using vortex::utils::types::Vector6d;

/**
 * @brief Converts an Eigen matrix to a dense matrix from CasADi
 *
 * @param &Matrix the matrix that is to be converted
 * @return The converted dense matrix
 */
inline casadi::DM eigen_to_dm(const Eigen::MatrixXd& matrix) {
    std::vector<double> data(matrix.data(), matrix.data() + matrix.size());
    casadi::Sparsity sp = casadi::Sparsity::dense(matrix.rows(), matrix.cols());
    return casadi::DM(sp, data, true);
}

/**
 * @brief Converts an Eigen vectorXd to a dense matrix from CasADi
 *
 * @param &vector the vector that is to be converted
 * @return The converted dense matrix
 */
inline casadi::DM eigen_to_dm(const Eigen::VectorXd& vector) {
    std::vector<double> data(vector.data(), vector.data() + vector.size());
    return casadi::DM(data);
}

/**
 * @brief Converts a CasADi dense matrix into an Eigen vectorXd
 *
 * @param &dense_matrix the dense_matrix that is to be converted
 * @return The converted vector
 */
inline Eigen::VectorXd dm_to_eigen_vector(const casadi::DM& dense_matrix) {
    std::vector<double> data = static_cast<std::vector<double>>(dense_matrix);
    Eigen::VectorXd out(static_cast<int>(data.size()));
    for (int i = 0; i < out.size(); ++i)
        out[i] = data[static_cast<size_t>(i)];
    return out;
}

#endif  // THRUST_ALLOCATOR_AUV__CASADI_UTILS_HPP_

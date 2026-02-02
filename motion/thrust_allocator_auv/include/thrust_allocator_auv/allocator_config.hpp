/**
 * @file allocator_config.hpp
 * @brief Contains the Allocator Config, which makes it easier
 * to choose allocator at runtime i..
 */

#ifndef THRUST_ALLOCATOR_AUV__ALLOCATOR_CONFIG_HPP_
#define THRUST_ALLOCATOR_AUV__ALLOCATOR_CONFIG_HPP_

#include <eigen3/Eigen/Eigen>

/**
 * @brief struct containing all configuration parameters for the different
 * solvers
 *
 * @param extended_thrust_matrix The extended thrust configuration matrix from
 * Fossen 2021 (11.16).
 * @param min_force Constraint on minimum amount of force from Fossen 2021
 * (11.38).
 * @param max_force Constraint on maximum amount of force from Fossen 2021
 * (11.38).
 * @param input_weight_matrix Diagonal matrix with input weights from Fossen
 * 2021 (11.27).
 * @param slack_weight_matrix Diagonal matrix with slack variable weights matrix
 * from Fossen 2021 (11.38).
 * @param beta The extended thrust configuration matrix from Fossen 2021
 * (11.38).
 */
struct AllocatorConfig {
    Eigen::MatrixXd extended_thrust_matrix;

    // Constraints
    Eigen::VectorXd min_force;
    Eigen::VectorXd max_force;

    // Weighting matrices
    Eigen::MatrixXd input_weight_matrix;
    Eigen::MatrixXd slack_weight_matrix;
    double beta;
};

#endif  // THRUST_ALLOCATOR_AUV__ALLOCATOR_CONFIG_HPP_

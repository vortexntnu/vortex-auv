/**
 * @file qp_allocator.hpp
 * @brief Contains the QP allocation method from Fossen 2021.
 */

#ifndef THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_
#define THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_

#include <eigen3/Eigen/Eigen>
#include "thrust_allocator_auv/allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"

class QPAllocator final : public Allocator {
    public:
    /**
     * @brief Constructor for the QPAllocator class.
     *
     * @param cfg all configuration parameters needed in the constructor
     */
    explicit QPAllocator(const AllocatorConfig& cfg);

    /**
     * @brief Calculates the necessary matrices to formulate the problem
     * as a convex QP.
     * @param cfg all configuration parameters needed in the constructor
     */
    void formulate_as_qp(const AllocatorConfig& cfg);

    /**
     * @brief Calculates the allocated thrust given the input torques using the
     * pre-calculated pseudoinverse matrix.
     *
     * @param tau The input torques as a vector.
     * @return The allocated thrust as a vector.
     */
    Eigen::VectorXd calculate_allocated_thrust(const Eigen::VectorXd& tau) override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Fossen 2021 (11.41)
    Eigen::VectorXd extended_state_vec_;
    Eigen::VectorXd extended_constraint_vec_;
    Eigen::MatrixXd square_term_matrix;
    Eigen::MatrixXd linear_term_matrix;
    Eigen::MatrixXd extended_equality_state_matrix;
    Eigen::MatrixXd extended_equality_constraint_matrix;
    Eigen::MatrixXd extended_inqeuality_state_matrix;
    Eigen::MatrixXd extended_inequality_constraint_matrix;

};

#endif  // THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_
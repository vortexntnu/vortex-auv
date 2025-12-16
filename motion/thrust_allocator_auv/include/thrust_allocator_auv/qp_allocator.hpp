/**
 * @file qp_allocator.hpp
 * @brief Contains the QP allocation method from Fossen 2021.
 */

#ifndef THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_
#define THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_

#include <eigen3/Eigen/Eigen>
#include "thrust_allocator_auv/allocator.hpp"

class QPAllocator final : public Allocator {
    public:
    /**
     * @brief Constructor for the QPAllocator class.
     *
     * @param cfg all configuration parameters needed in the constructor
     */
    explicit QPAllocator(const AllocatorConfig& cfg);

    /**
     * @brief Calculates the allocated thrust given the input torques using the
     * pre-calculated pseudoinverse matrix.
     *
     * @param tau The input torques as a vector.
     * @return The allocated thrust as a vector.
     */
    Eigen::VectorXd calculate_allocated_thrust(const Eigen::VectorXd& tau) override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXd extended_thrust_matrix_;
    Eigen::MatrixXd input_weght_matrix_;
};

#endif  // THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_
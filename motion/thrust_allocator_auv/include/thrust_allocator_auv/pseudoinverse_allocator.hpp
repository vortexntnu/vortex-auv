/**
 * @file pseudoinverse_allocator.hpp
 * @brief Contains the PseudoinverseAllocator class, which implements the
 * unweighted pseudoinverse-based allocator described in e.g. Fossen 2011
 * Handbook of Marine Craft Hydrodynamics and Motion Control (chapter 12.3.2).
 */

#ifndef THRUST_ALLOCATOR_AUV__PSEUDOINVERSE_ALLOCATOR_HPP_
#define THRUST_ALLOCATOR_AUV__PSEUDOINVERSE_ALLOCATOR_HPP_

#include <eigen3/Eigen/Eigen>

/**
 * @brief The PseudoinverseAllocator class calculates the allocated thrust given
 * the input torques using the pseudoinverse allocator.
 */
class PseudoinverseAllocator {
   public:
    /**
     * @brief Constructor for the PseudoinverseAllocator class.
     *
     * @param thrust_matrix_pseudoinverse_ The pseudoinverse of the thruster configuration matrix.
     * @param input_weight_matrix Diagonal matrix with weights from Fossen (11.27).
     * @param extended_thrust_matrix The extended thrust configuration matrix Fossen (11.16).
     */
    explicit PseudoinverseAllocator::PseudoinverseAllocator(const Eigen::MatrixXd& extended_thrust_matrix,
    const Eigen::MatrixXd& input_weight_matrix);

    /**
     * @brief Calculates the pseudoinverse of the given matrix.
     *
     * @param T The extended thrust configuration matrix from Fossen (11.16).
     * @param W The input weight matrix from Fossen (11.27).
     * @throws char* if the pseudoinverse is invalid.
     * @return The pseudoinverse of the given matrix.
     */
   Eigen::MatrixXd calculate_pseudoinverse(const Eigen::MatrixXd& T, const Eigen::MatrixXd& W) ;

    /**
     * @brief Calculates the allocated thrust given the input torques using the
     * pseudoinverse allocator.
     *
     * @param tau The input torques as a vector.
     * @return The allocated thrust as a vector.
     */
    Eigen::VectorXd calculate_allocated_thrust(const Eigen::VectorXd& tau);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXd extended_thrust_matrix_;
    Eigen::MatrixXd input_weght_matrix_;
    Eigen::MatrixXd thrust_matrix_pseudoinverse_;
};

#endif  // THRUST_ALLOCATOR_AUV__PSEUDOINVERSE_ALLOCATOR_HPP_

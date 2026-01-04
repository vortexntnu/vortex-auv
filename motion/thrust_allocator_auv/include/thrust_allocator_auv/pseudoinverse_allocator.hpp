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
     * @param T_pinv The pseudoinverse of the thruster configuration matrix.
     */
    explicit PseudoinverseAllocator(const Eigen::MatrixXd& T_pinv);

    /**
     * @brief Calculates the allocated thrust given the input torques using the
     * pseudoinverse allocator.
     *
     * @param tau The input torques as a vector.
     * @return The allocated thrust as a vector.
     */
    Eigen::VectorXd calculate_allocated_thrust(const Eigen::VectorXd& tau);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXd T_pinv;
};

#endif  // THRUST_ALLOCATOR_AUV__PSEUDOINVERSE_ALLOCATOR_HPP_

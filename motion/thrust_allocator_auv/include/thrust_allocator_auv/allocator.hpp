/**
 * @file allocator.hpp
 * @brief Contains the Allocator base class, which the final
 * allocators will be derived from
 */

#ifndef THRUST_ALLOCATOR_AUV__ALLOCATOR_HPP_
#define THRUST_ALLOCATOR_AUV__ALLOCATOR_HPP_

#include <eigen3/Eigen/Eigen>

/**
 * @brief The Allocator class structure that the solvers will inherit and override.
 */
class Allocator {
   public:
    /**
     * @brief Constructor for the Allocator class.
     */
    
    virtual ~Allocator() = default;

    /**
     * @brief Calculates the allocated thrust given the input torques using the
     * pre-calculated pseudoinverse matrix.
     *
     * @param tau The input torques as a vector.
     * @return The allocated thrust as a vector.
     */
    virtual Eigen::VectorXd calculate_allocated_thrust(const Eigen::VectorXd& tau) = 0;
};

#endif  // THRUST_ALLOCATOR_AUV__ALLOCATOR_HPP_

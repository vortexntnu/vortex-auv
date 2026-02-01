/**
 * @file qp_allocator.hpp
 * @brief Contains the QP allocation method from Fossen 2021.
 */

#ifndef THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_
#define THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_

#include "casadi/casadi.hpp"
#include "thrust_allocator_auv/allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

class QPAllocator final : public Allocator {
public:
  /**
   * @brief Constructor for the QPAllocator class.
   *
   * @param cfg all configuration parameters needed in the constructor
   */
  explicit QPAllocator(const AllocatorConfig &cfg);

  /**
   * @brief Calculates the allocated thrust using CasADi's conic library
   * @param tau The input torques as a vector.
   * @return The allocated thrust as a vector.
   */
  Eigen::VectorXd
  calculate_allocated_thrust(const Eigen::VectorXd &tau) override;

private:
  /**
   * @brief Calculates the necessary matrices to formulate the problem
   * as a convex QP.
   * @param cfg all configuration parameters needed in the constructor
   */
  void formulate_as_qp(const AllocatorConfig &cfg);

  /**
   * @brief transforms the problem from QP standardform to CasADi formulation
   */
  void formulate_as_qp_CasADi(const AllocatorConfig &cfg);

  int degrees_of_freedom_;
  int number_of_thrusters;
  // Fossen formulation of QP, Fossen 2021 (11.41)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd extended_state_vec_;                  // z in fossen's book
  Eigen::VectorXd extended_constraint_vec_;             // p in fossen's book
  Eigen::MatrixXd square_term_matrix_;                  // Phi in fossen's book
  Eigen::MatrixXd mixed_term_matrix_;                   // R in fossen's book
  Eigen::MatrixXd extended_equality_state_matrix_;      // A1 in fossen's book
  Eigen::MatrixXd extended_equality_constraint_matrix_; // C1 in fossen's boo
  Eigen::MatrixXd extended_inequality_state_matrix_;    // A2 in fossen's boo
  Eigen::MatrixXd extended_inequality_constraint_matrix_; // C2 in fossen's boo
  int beta_;

  // CasADi fomulation of QP
  // TODO: Remember to implement warm starting after testing with QP
  Eigen::MatrixXd stacked_constraint_matrix_;
  casadi::Function casadi_qp_solver_;
  casadi::Sparsity H_sparsity_pattern_;
  casadi::Sparsity A_sparsity_pattern_;
  casadi::DM H_matrix;
  casadi::DM g_matrix;
  casadi::DM A_matrix;
  casadi::DM lower_bound_constraint_matrix;
  casadi::DM upper_bound_constraint_matrix;
  casadi::DM lower_bound_state_matrix;
  casadi::DM upper_bound_state_matrix;
  bool casadi_solver_initialized_;
};

#endif // THRUST_ALLOCATOR_AUV__QP_ALLOCATOR_HPP_

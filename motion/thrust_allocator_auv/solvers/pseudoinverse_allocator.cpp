#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

PseudoinverseAllocator::PseudoinverseAllocator(
    const AllocatorConfig &allocator_config)
    : extended_thrust_matrix_(allocator_config.extended_thrust_matrix),
      input_weight_matrix_(allocator_config.input_weight_matrix) {
  thrust_matrix_pseudoinverse_ =
      calculate_pseudoinverse(extended_thrust_matrix_, input_weight_matrix_);
}

Eigen::MatrixXd
PseudoinverseAllocator::calculate_pseudoinverse(const Eigen::MatrixXd &T,
                                                const Eigen::MatrixXd &W) {
  Eigen::MatrixXd pseudoinverse =
      W.inverse() * T.transpose() * (T * W.inverse() * T.transpose()).inverse();
  if (is_invalid_matrix(pseudoinverse)) {
    throw std::runtime_error("Invalid Pseudoinverse Calculated");
  }
  return pseudoinverse;
}

std::optional<Eigen::VectorXd>
PseudoinverseAllocator::calculate_allocated_thrust(const Eigen::VectorXd &tau) {
  Eigen::VectorXd u = thrust_matrix_pseudoinverse_ * tau;
  return u;
}

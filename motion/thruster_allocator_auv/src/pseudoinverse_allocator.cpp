#include "thruster_allocator_auv/pseudoinverse_allocator.hpp"

PseudoinverseAllocator::PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv)
    : T_pinv(T_pinv) {}

Eigen::VectorXd
PseudoinverseAllocator::calculateAllocatedThrust(const Eigen::VectorXd &tau) {
  Eigen::VectorXd u = T_pinv * tau;
  return u;
}
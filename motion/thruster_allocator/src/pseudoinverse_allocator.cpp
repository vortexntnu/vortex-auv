#include "vortex_allocator/pseudoinverse_allocator.h"

PseudoinverseAllocator::PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv)
  : T_pinv(T_pinv) {}

Eigen::VectorXd PseudoinverseAllocator::compute(const Eigen::VectorXd &tau)
{
  Eigen::VectorXd u = T_pinv * tau;
  return u;
}

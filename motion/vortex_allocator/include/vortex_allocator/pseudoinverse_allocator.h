// Implement the unweighted pseudoinverse-based allocator described in e.g.
// Fossen 2011 Handbook of Marine Craft Hydrodynamics and Motion Control
// (chapter 12.3.2).

#ifndef VORTEX_ALLOCATOR_PSEUDOINVERSE_ALLOCATOR_H
#define VORTEX_ALLOCATOR_PSEUDOINVERSE_ALLOCATOR_H

#include <Eigen/Dense>

class PseudoinverseAllocator
{
public:
  explicit PseudoinverseAllocator(const Eigen::MatrixXd &T_pinv);
  Eigen::VectorXd compute(const Eigen::VectorXd &tau);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::MatrixXd T_pinv;
};

#endif  // VORTEX_ALLOCATOR_PSEUDOINVERSE_ALLOCATOR_H

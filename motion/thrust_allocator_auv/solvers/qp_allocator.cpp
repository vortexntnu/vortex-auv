#include "thrust_allocator_auv/qp_allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"

QPAllocator::QPAllocator(const AllocatorConfig &cfg) { formulate_as_qp(cfg); };

void QPAllocator::formulate_as_qp(const AllocatorConfig &cfg) {

  const int r = cfg.input_weight_matrix.rows(); // number of thrusters = 8
  const int n = cfg.slack_weight_matrix.rows(); // degrees of freedom = 6

  // create and set the square term matrix.
  Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(r + n + 1, r + n + 1);
  phi.block(0, 0, r, r) = cfg.input_weight_matrix;
  phi.block(r, r, n, n) = cfg.slack_weight_matrix;

  this->square_term_matrix_ = phi;

  // construct and set the linear term matrix.
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(r + n + 1, n + 2 * r + 1);
  R(r + n, n + 2 * r) = 1;

  this->linear_term_matrix_ = R;

  // construct and set the matrix of equality constraints w.r.t state variable
  // z.
  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(n, r + n + 1);
  A1.block(0, 0, n, r) = cfg.extended_thrust_matrix;
  A1.block(0, r, n, n) = -Eigen::MatrixXd::Identity(n, n);

  this->extended_equality_state_matrix_ = A1;

  // construct and set the matrix of inequalities w.r.t state variable z.
  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(4 * r, r + n + 1);
  A2.block(0, 0, r, r) = -Eigen::MatrixXd::Identity(r, r);
  A2.block(r, 0, r, r) = Eigen::MatrixXd::Identity(r, r);
  A2.block(2 * r, 0, r, r) = Eigen::MatrixXd::Identity(r, r);
  A2.block(3 * r, 0, r, r) = -Eigen::MatrixXd::Identity(r, r);
  A2.block(2 * r, r + n, r, 1) = -Eigen::VectorXd::Ones(r);
  A2.block(3 * r, r + n, r, 1) = -Eigen::VectorXd::Ones(r);

  this->extended_inequality_state_matrix_ = A2;

  // construct and set the matrix of equality constraints w.r.t decision
  // variable p.
  Eigen::MatrixXd C1 = Eigen::MatrixXd::Zero(n, n + 2 * r + 1);
  C1.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);

  this->extended_equality_constraint_matrix_ = C1;

  // construct and set the matrix of inequality constraints w.r.t decision
  // variable p.
  Eigen::MatrixXd C2 = Eigen::MatrixXd::Zero(4 * r, n + 2 * r + 1);
  C2.block(0, n, r, r) = -Eigen::MatrixXd::Identity(r, r);
  C2.block(r, n + r, r, r) = Eigen::MatrixXd::Identity(r, r);

  this->extended_inequality_constraint_matrix_ = C2;
};

Eigen::VectorXd
QPAllocator::calculate_allocated_thrust(const Eigen::VectorXd &tau) {
  return tau;
};

#include "thrust_allocator_auv/qp_allocator.hpp"
#include "casadi/casadi.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

QPAllocator::QPAllocator(const AllocatorConfig &cfg)
    : casadi_solver_initialized_(false) {
  formulate_as_qp(cfg);
  formulate_as_qp_CasADi(cfg);
};

void QPAllocator::formulate_as_qp(const AllocatorConfig &cfg) {

  const int r = cfg.input_weight_matrix.rows(); // number of thrusters = 8
  const int n = cfg.slack_weight_matrix.rows(); // degrees of freedom = 6
  this->number_of_thrusters = r;
  this->degrees_of_freedom_ = n;

  // create and set the square term matrix.
  Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(r + n + 1, r + n + 1);
  phi.block(0, 0, r, r) = cfg.input_weight_matrix;
  phi.block(r, r, n, n) = cfg.slack_weight_matrix;

  this->square_term_matrix_ = phi;

  // construct and set the mixted term matrix.
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(r + n + 1, n + 2 * r + 1);
  R(r + n, n + 2 * r) = 1;

  this->mixed_term_matrix_ = R;

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

  // get beta value from cfg
  this->beta_ = cfg.beta;

  // fill the constant part of the P vector (tau gets filled every iteration in
  // calculate_allocated_thrust)
  extended_constraint_vec_.setZero(n + 2 * r + 1);
  extended_constraint_vec_.segment(n, r) = cfg.min_force;
  extended_constraint_vec_.segment(n + r, r) = cfg.max_force;
  extended_constraint_vec_(n + 2 * r) = beta_;
};

void QPAllocator::formulate_as_qp_CasADi(const AllocatorConfig &cfg) {

  const int r = cfg.input_weight_matrix.rows(); // number of thrusters = 8
  const int n = cfg.slack_weight_matrix.rows(); // degrees of freedom = 6

  const int decision_variable_dimension =
      static_cast<int>(square_term_matrix_.rows());

  const int equality_constraint_count =
      static_cast<int>(extended_equality_state_matrix_.rows());

  const int inequality_constraint_count =
      static_cast<int>(extended_inequality_state_matrix_.rows());

  const int total_constraint_count =
      equality_constraint_count + inequality_constraint_count;

  H_sparsity_pattern_ = casadi::Sparsity::dense(decision_variable_dimension,
                                                decision_variable_dimension);

  A_sparsity_pattern_ = casadi::Sparsity::dense(total_constraint_count,
                                                decision_variable_dimension);

  casadi_qp_solver_ =
      casadi::conic("thrust_allocation_qp", "qrqp",
                    {{"h", H_sparsity_pattern_}, {"a", A_sparsity_pattern_}});

  stacked_constraint_matrix_.setZero(total_constraint_count,
                                     decision_variable_dimension);
  H_matrix = casadi::DM::zeros(decision_variable_dimension,
                               decision_variable_dimension);
  g_matrix = casadi::DM::zeros(decision_variable_dimension, 1);

  A_matrix =
      casadi::DM::zeros(total_constraint_count, decision_variable_dimension);

  lower_bound_constraint_matrix = casadi::DM::zeros(total_constraint_count, 1);
  upper_bound_constraint_matrix = casadi::DM::zeros(total_constraint_count, 1);

  lower_bound_state_matrix = casadi::DM::zeros(decision_variable_dimension, 1);
  upper_bound_state_matrix = casadi::DM::zeros(decision_variable_dimension, 1);

  {
    const double inf = std::numeric_limits<double>::infinity();

    std::vector<double> lbx(decision_variable_dimension, -inf);
    std::vector<double> ubx(decision_variable_dimension, inf);

    lower_bound_state_matrix = casadi::DM(lbx);
    upper_bound_state_matrix = casadi::DM(ubx);
  }

  stacked_constraint_matrix_.topRows(equality_constraint_count) =
      extended_equality_state_matrix_;
  stacked_constraint_matrix_.bottomRows(inequality_constraint_count) =
      extended_inequality_state_matrix_;

  H_matrix = eigen_to_dm(square_term_matrix_);
  A_matrix = eigen_to_dm(stacked_constraint_matrix_);

  casadi_solver_initialized_ = true;
}

Eigen::VectorXd
QPAllocator::calculate_allocated_thrust(const Eigen::VectorXd &tau) {
  if (!casadi_solver_initialized_) {
    throw std::runtime_error("CasADi QP solver not initialized "
                             "(formulate_as_qp_CasADi not called).");
  }

  if (tau.size() != degrees_of_freedom_) {
    throw std::runtime_error("tau dimension mismatch");
  }

  const int equality_constraint_count =
      static_cast<int>(extended_equality_state_matrix_.rows());
  const int inequality_constraint_count =
      static_cast<int>(extended_inequality_state_matrix_.rows());
  const int total_constraint_count =
      equality_constraint_count + inequality_constraint_count;

  // fill the part of p that corresponds to tau
  extended_constraint_vec_.head(degrees_of_freedom_) = tau;

  // create the combination R*p after filling in tau
  Eigen::VectorXd linear_cost_vector =
      mixed_term_matrix_ * extended_constraint_vec_;

  // g_matrix will be Rp such that the mixed term end up being x.T @ R @ p
  g_matrix = eigen_to_dm(linear_cost_vector);

  Eigen::VectorXd equality_rhs =
      extended_equality_constraint_matrix_ * extended_constraint_vec_;
  Eigen::VectorXd inequality_rhs =
      extended_inequality_constraint_matrix_ * extended_constraint_vec_;

  //  Build lba/uba
  Eigen::VectorXd lower_bound_constraints(total_constraint_count);
  Eigen::VectorXd upper_bound_constraints(total_constraint_count);

  // equality: A1 z = C1 p
  lower_bound_constraints.head(equality_constraint_count) = equality_rhs;
  upper_bound_constraints.head(equality_constraint_count) = equality_rhs;

  // inequality: -inf <= A2 z <= C2 p
  lower_bound_constraints.tail(inequality_constraint_count)
      .setConstant(-std::numeric_limits<double>::infinity());
  upper_bound_constraints.tail(inequality_constraint_count) = inequality_rhs;

  lower_bound_constraint_matrix = eigen_to_dm(lower_bound_constraints);
  upper_bound_constraint_matrix = eigen_to_dm(upper_bound_constraints);

  // Feed everything into CasADi QP solver
  casadi::DMDict arg{
      {"h", H_matrix},
      {"g", g_matrix},
      {"a", A_matrix},
      {"lba", lower_bound_constraint_matrix},
      {"uba", upper_bound_constraint_matrix},
      {"lbx", lower_bound_state_matrix},
      {"ubx", upper_bound_state_matrix},
  };

  casadi::DMDict res = casadi_qp_solver_(arg);

  // Extract the decision-variable solution z from CasADi
  casadi::DM extended_state_solution = res.at("x");

  // Convert CasADi DM -> raw doubles
  std::vector<double> extended_state_solution_raw =
      static_cast<std::vector<double>>(extended_state_solution);

  // Map raw doubles -> Eigen vector (copy if you want it to own memory)
  Eigen::VectorXd extended_state_solution_final = Eigen::Map<Eigen::VectorXd>(
      extended_state_solution_raw.data(),
      static_cast<int>(extended_state_solution_raw.size()));

  // Return thruster commands (first r entries of z)
  return extended_state_solution_final.head(number_of_thrusters);
};

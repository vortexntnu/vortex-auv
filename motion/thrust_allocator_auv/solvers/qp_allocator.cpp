#include "thrust_allocator_auv/qp_allocator.hpp"
#include "casadi/casadi.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

#ifdef NDEBUG
constexpr bool casadi_debug = false; // Release
#else
constexpr bool casadi_debug = true; // Debug
#endif

QPAllocator::QPAllocator(const AllocatorConfig &allocator_config)
    : casadi_solver_initialized_(false) {
  formulate_as_qp(allocator_config);
  formulate_as_qp_casadi();
};

void QPAllocator::formulate_as_qp(const AllocatorConfig &allocator_config) {
  const int r =
      allocator_config.input_weight_matrix.rows(); // number of thrusters
  const int n =
      allocator_config.slack_weight_matrix.rows(); // degrees of freedom

  // Save important constants from allocator_config that will be used in
  // building CasADi formulation
  this->number_of_thrusters_ = r;
  this->degrees_of_freedom_ = n;
  this->max_force_ = allocator_config.max_force[0];
  // create and set the square term matrix.
  Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(r + n, r + n);
  phi.block(0, 0, r, r) = allocator_config.input_weight_matrix;
  phi.block(r, r, n, n) = allocator_config.slack_weight_matrix;

  // Save hessian matrix as member variable
  this->square_term_matrix_ = phi;

  // construct and set the mixed term matrix.
  this->mixed_term_matrix_ = Eigen::MatrixXd::Zero(r + n, n + 2 * r);

  // construct and set the matrix of equality constraints w.r.t state variable
  // z.
  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(n, r + n);
  A1.block(0, 0, n, r) = allocator_config.extended_thrust_matrix;
  A1.block(0, r, n, n) = -Eigen::MatrixXd::Identity(n, n);
  this->extended_equality_state_matrix_ = A1;

  // construct and set the matrix of inequalities w.r.t state variable z.
  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(2 * r, r + n);
  A2.block(0, 0, r, r) = -Eigen::MatrixXd::Identity(r, r);
  A2.block(r, 0, r, r) = Eigen::MatrixXd::Identity(r, r);
  this->extended_inequality_state_matrix_ = A2;

  // construct and set the matrix of equality constraints w.r.t decision
  // variable p.
  Eigen::MatrixXd C1 = Eigen::MatrixXd::Zero(n, n + 2 * r);
  C1.block(0, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
  this->extended_equality_constraint_matrix_ = C1;

  // construct and set the matrix of inequality constraints w.r.t decision
  // variable p.
  Eigen::MatrixXd C2 = Eigen::MatrixXd::Zero(2 * r, n + 2 * r);
  C2.block(0, n, r, r) = -Eigen::MatrixXd::Identity(r, r);
  C2.block(r, n + r, r, r) = Eigen::MatrixXd::Identity(r, r);
  this->extended_inequality_constraint_matrix_ = C2;

  // fill the constant part of the P vector (tau gets filled every iteration
  // in calculate_allocated_thrust)
  extended_constraint_vec_.setZero(n + 2 * r);
  extended_constraint_vec_.segment(n, r) = allocator_config.min_force;
  extended_constraint_vec_.segment(n + r, r) = allocator_config.max_force;
  spdlog::info("Successfully formulated as QP standardform");
};

void QPAllocator::formulate_as_qp_casadi() {
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

  casadi::Dict opts;
  opts["print_time"] = casadi_debug;
  opts["verbose"] = casadi_debug;
  opts["print_iter"] = casadi_debug;
  opts["print_header"] = casadi_debug;
  opts["print_info"] = casadi_debug;

  casadi_qp_solver_ = casadi::conic(
      "thrust_allocation_qp", "qrqp",
      {{"h", H_sparsity_pattern_}, {"a", A_sparsity_pattern_}}, opts);

  stacked_constraint_matrix_.setZero(total_constraint_count,
                                     decision_variable_dimension);
  H_matrix = casadi::DM::zeros(decision_variable_dimension,
                               decision_variable_dimension);

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

  // Set the correct H and A matrix utilizing the
  H_matrix = eigen_to_dm(square_term_matrix_);
  A_matrix = eigen_to_dm(stacked_constraint_matrix_);

  // Preallocate the size of the previous solution.
  previous_solution_ = casadi::DM::zeros(decision_variable_dimension, 1);
  have_previous_solution_ = false;

  spdlog::info("Successfully formulated into valid CasADi formulation");
  casadi_solver_initialized_ = true;
}

std::optional<Eigen::VectorXd>
QPAllocator::calculate_allocated_thrust(const Eigen::VectorXd &tau) {
  if (!casadi_solver_initialized_) {
    return std::nullopt;
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

  // fill the part of p that corresponds to the generalized forces
  Eigen::VectorXd tau_scaled = normalizeWrenchVector(tau, max_force_);
  extended_constraint_vec_.head(degrees_of_freedom_) = tau_scaled;

  // DO NOT want beta term from fossen as 6DOF does not lend well
  g_matrix = casadi::DM::zeros(static_cast<int>(square_term_matrix_.rows()), 1);

  Eigen::VectorXd equality_rhs =
      extended_equality_constraint_matrix_ * extended_constraint_vec_;
  Eigen::VectorXd inequality_rhs =
      extended_inequality_constraint_matrix_ * extended_constraint_vec_;

  //  Build lower and upper bounds constraint vectors
  Eigen::VectorXd lower_bound_constraints(total_constraint_count);
  Eigen::VectorXd upper_bound_constraints(total_constraint_count);

  // equality: A1 z = C1 p
  lower_bound_constraints.head(equality_constraint_count) = equality_rhs;
  upper_bound_constraints.head(equality_constraint_count) = equality_rhs;

  // inequality: -inf <= A2 z <= C2 p
  lower_bound_constraints.tail(inequality_constraint_count)
      .setConstant(-std::numeric_limits<double>::infinity());
  upper_bound_constraints.tail(inequality_constraint_count) = inequality_rhs;

  // cast the matrices from Eigen to Double Matrix for CasADi
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

  // If we have access to a previous solution we warm start the solver
  if (warm_start_enabled_ && have_previous_solution_) {
    arg["x0"] = previous_solution_;
  }

  // Use the CasADi solver
  casadi::DMDict res = casadi_qp_solver_(arg);

  // Extract the decision-variable solution z from CasADi
  casadi::DM extended_state_solution = res.at("x");

  // Remember the solution and enable warm starting
  previous_solution_ = res.at("x");
  have_previous_solution_ = true;

  // Convert CasADi DM -> raw doubles
  std::vector<double> extended_state_solution_raw =
      static_cast<std::vector<double>>(extended_state_solution);

  // Map raw doubles -> Eigen vector (copy if you want it to own memory)
  Eigen::VectorXd extended_state_solution_final = Eigen::Map<Eigen::VectorXd>(
      extended_state_solution_raw.data(),
      static_cast<int>(extended_state_solution_raw.size()));

  // Return thruster commands (first r entries of z)
  return extended_state_solution_final.head(number_of_thrusters_);
};

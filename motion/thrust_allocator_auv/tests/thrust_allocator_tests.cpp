#include <eigen3/Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "thrust_allocator_auv/allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/allocator_factory.hpp"
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"
#include "thrust_allocator_auv/qp_allocator.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"
#include "vortex/utils/types.hpp"
#include <vortex/utils/math.hpp>

// ------------------- Constants -------------------
const double tau_tol = 7e-1; // Tolerance value for how far the reconstructed
                             // wrench vector by the thruster allocator module
                             // can maximally be from the desired wrench vector

using vortex::utils::types::Vector6d;

void fill_input_from_double_array(Vector6d &tau, const double *double_array) {
  tau << double_array[0], double_array[1], double_array[2], double_array[3],
      double_array[4], double_array[5];
}

void print_generalized_force(const Vector6d &tau) {
  spdlog::info("Tau values:");
  spdlog::info("Surge force: {}", tau[0]);
  spdlog::info("Sway force: {}", tau[1]);
  spdlog::info("Heave force: {}", tau[2]);
  spdlog::info("Roll moment: {}", tau[3]);
  spdlog::info("Pitch moment: {}", tau[4]);
  spdlog::info("Yaw moment: {}", tau[5]);
}

//  When looking at the AUV from above (ORCA)
//
//         front
//        |======|
//   |=7↗=|      |=0↖=|
//   |    |      |    |
//   | 6• |      | 1• |
//   |    |      |    |
//   |    |      |    |
//   | 5• |      | 2• |
//   |    |      |    |
//   |=4↖=|==||==|=3↗=|
//

void print_input(const Eigen::VectorXd &thruster_forces) {
  spdlog::info("Input values:");
  spdlog::info("thruster_0: {}", thruster_forces[0]);
  spdlog::info("thruster_1: {}", thruster_forces[1]);
  spdlog::info("thruster_2: {}", thruster_forces[2]);
  spdlog::info("thruster_3: {}", thruster_forces[3]);
  spdlog::info("thruster_4: {}", thruster_forces[4]);
  spdlog::info("thruster_5: {}", thruster_forces[5]);
  spdlog::info("thruster_6: {}", thruster_forces[6]);
  spdlog::info("thruster_7: {}", thruster_forces[7]);
}

static void expect_valid_qp_output(const Eigen::VectorXd &u,
                                   const AllocatorConfig &cfg,
                                   double tol = 1e-9) {
  ASSERT_EQ(u.size(), cfg.extended_thrust_matrix.cols());
  ASSERT_TRUE(u.allFinite());

  for (int i = 0; i < u.size(); ++i) {
    EXPECT_LE(u[i], cfg.max_force[i] + tol);
    EXPECT_GE(u[i], cfg.min_force[i] - tol);
  }
}

static void expect_group_dominance(const Eigen::VectorXd &u,
                                   const std::vector<int> &active,
                                   const std::vector<int> &quiet,
                                   double ratio = 10.0) {
  double sum_active = 0.0, sum_quiet = 0.0;
  for (int i : active) {
    sum_active += std::abs(u[i]);
  }
  for (int i : quiet) {
    sum_quiet += std::abs(u[i]);
  }
  EXPECT_GT(sum_active, ratio * (sum_quiet + 1e-12));
}

static Eigen::VectorXd require_thrust(std::unique_ptr<Allocator> &allocator,
                                      const Eigen::VectorXd &tau) {
  auto u_opt = allocator->calculate_allocated_thrust(tau);

  EXPECT_TRUE(u_opt.has_value()) << "Allocator returned nullopt";

  if (!u_opt.has_value()) {
    return Eigen::VectorXd();
  }

  return *u_opt;
}

AllocatorConfig load_allocator_config(const std::string &yaml_path) {
  YAML::Node params = YAML::LoadFile(yaml_path)["/**"]["ros__parameters"];
  YAML::Node physical = params["physical"];
  YAML::Node propulsion = params["propulsion"];
  YAML::Node thrusters = propulsion["thrusters"];
  YAML::Node constraints = thrusters["constraints"];

  const int num_thrusters = thrusters["num"].as<int>();
  const int degrees_of_freedom = propulsion["dofs"]["num"].as<int>();

  Eigen::MatrixXd thruster_force_directions = double_array_to_eigen_matrix(
      thrusters["thruster_force_direction"].as<std::vector<double>>(), 3,
      num_thrusters);

  Eigen::MatrixXd thruster_positions = double_array_to_eigen_matrix(
      thrusters["thruster_position"].as<std::vector<double>>(), 3,
      num_thrusters);

  Eigen::Vector3d center_of_mass = double_array_to_eigen_vector3d(
      physical["center_of_mass"].as<std::vector<double>>());

  Eigen::MatrixXd thrust_configuration_matrix =
      calculate_thrust_configuration_matrix(thruster_force_directions,
                                            thruster_positions, center_of_mass);

  const double min_thruster_force = constraints["min_force"].as<double>();
  const double max_thruster_force = constraints["max_force"].as<double>();

  Eigen::VectorXd min_force =
      Eigen::VectorXd::Constant(num_thrusters, min_thruster_force);
  Eigen::VectorXd max_force =
      Eigen::VectorXd::Constant(num_thrusters, max_thruster_force);

  std::vector<double> input_weight_values =
      constraints["input_matrix_weights"].as<std::vector<double>>();

  std::vector<double> slack_weight_values =
      constraints["slack_matrix_weights"].as<std::vector<double>>();

  Eigen::VectorXd input_weight_diagonal = Eigen::Map<const Eigen::VectorXd>(
      input_weight_values.data(), num_thrusters);

  Eigen::VectorXd slack_weight_diagonal = Eigen::Map<const Eigen::VectorXd>(
      slack_weight_values.data(), degrees_of_freedom);

  AllocatorConfig config;
  config.extended_thrust_matrix = thrust_configuration_matrix;
  config.min_force = min_force;
  config.max_force = max_force;
  config.input_weight_matrix = input_weight_diagonal.asDiagonal();
  config.slack_weight_matrix = slack_weight_diagonal.asDiagonal();

  return config;
}

class ThrustAllocatorYamlTests : public ::testing::Test {
protected:
  AllocatorConfig allocator_config;

  void SetUp() override { allocator_config = load_allocator_config(YAML_PATH); }
};

TEST_F(ThrustAllocatorYamlTests, T00_CorrectlyPulledConfigFromYamlFile) {
  ASSERT_EQ(allocator_config.extended_thrust_matrix.rows(), 6);
  ASSERT_EQ(allocator_config.extended_thrust_matrix.cols(), 8);

  ASSERT_EQ(allocator_config.min_force.size(),
            allocator_config.max_force.size());
  ASSERT_EQ(allocator_config.extended_thrust_matrix.cols(),
            allocator_config.min_force.size());

  ASSERT_FALSE(is_invalid_matrix(allocator_config.extended_thrust_matrix));
}

class PseudoinverseAllocatorTests : public ::testing::Test {
protected:
  AllocatorConfig allocator_config;
  std::unique_ptr<Allocator> allocator;

  void SetUp() override {
    allocator_config = load_allocator_config(YAML_PATH);
    allocator = Factory::make_allocator("pseudoinverse", allocator_config);
  }
};

class QPAllocatorTests : public ::testing::Test {
protected:
  AllocatorConfig allocator_config;
  std::unique_ptr<Allocator> allocator;

  void SetUp() override {
    allocator_config = load_allocator_config(YAML_PATH);
    allocator = Factory::make_allocator("qp", allocator_config);
  }
};

// ----------------------- pseudoinverse allocator tests -----------------------

TEST_F(PseudoinverseAllocatorTests, P01_ClampingWorks) {
  Vector6d tau;
  double fill[6] = {100000.0, 100000.0, 100000.0, 100000.0, 100000.0, 100000.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  saturate_vector_values(u, -100.0, 100.0);
  print_input(u);

  EXPECT_LE(u[7], 100.0);
  EXPECT_GE(u[7], -100.0);
  EXPECT_LE(u[6], 100.0);
  EXPECT_GE(u[6], -100.0);
  EXPECT_LE(u[5], 100.0);
  EXPECT_GE(u[5], -100.0);
  EXPECT_LE(u[4], 100.0);
  EXPECT_GE(u[4], -100.0);
  EXPECT_LE(u[3], 100.0);
  EXPECT_GE(u[3], -100.0);
  EXPECT_LE(u[2], 100.0);
  EXPECT_GE(u[2], -100.0);
  EXPECT_LE(u[1], 100.0);
  EXPECT_GE(u[1], -100.0);
  EXPECT_LE(u[0], 100.0);
  EXPECT_GE(u[0], -100.0);
}

TEST_F(PseudoinverseAllocatorTests, P02_Pseudo_ZeroWrenchGivesZeroThrust) {
  Vector6d tau = Vector6d::Zero();
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_NEAR(u.norm(), 0.0, 1e-9);
}

TEST_F(PseudoinverseAllocatorTests,
       P03_Pseudo_ForwardWrenchGivesForwardThrust) {
  Vector6d tau;
  double fill[6] = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_GT(u[7], 0.0);
  EXPECT_GT(u[4], 0.0);
  EXPECT_GT(u[3], 0.0);
  EXPECT_GT(u[0], 0.0);
}

TEST_F(PseudoinverseAllocatorTests,
       P04_Pseudo_BackwardWrenchGivesBackwardThrust) {
  Vector6d tau;
  double fill[6] = {-100.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_LT(u[7], 0.0);
  EXPECT_LT(u[4], 0.0);
  EXPECT_LT(u[3], 0.0);
  EXPECT_LT(u[0], 0.0);
}

TEST_F(PseudoinverseAllocatorTests,
       P05_Pseudo_SidewayWrenchGivesSidewayThrust) {
  Vector6d tau;
  double fill[6] = {0.0, 100.0, 0.0, 0.0, 0.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_GT(u[7], 0.0);
  EXPECT_LT(u[4], 0.0);
  EXPECT_GT(u[3], 0.0);
  EXPECT_LT(u[0], 0.0);
}

TEST_F(PseudoinverseAllocatorTests,
       P06_Pseudo_DownwardWrenchGivesDownwardThrust) {
  Vector6d tau;
  double fill[6] = {0.0, 0.0, 100.0, 0.0, 0.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_GT(u[6], 0.0);
  EXPECT_GT(u[5], 0.0);
  EXPECT_GT(u[2], 0.0);
  EXPECT_GT(u[1], 0.0);
}

TEST_F(PseudoinverseAllocatorTests, P07_Pseudo_UpwardWrenchGivesUpwardThrust) {
  Vector6d tau;
  double fill[6] = {0.0, 0.0, -100.0, 0.0, 0.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_LT(u[6], 0.0);
  EXPECT_LT(u[5], 0.0);
  EXPECT_LT(u[2], 0.0);
  EXPECT_LT(u[1], 0.0);
}

TEST_F(PseudoinverseAllocatorTests,
       P08_Pseudo_PositiveRollWrenchInducesPositiveRollMoment) {
  Vector6d tau;
  double fill[6] = {0.0, 0.0, 0.0, 100.0, 0.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_LT(u[6], 0.0);
  EXPECT_LT(u[5], 0.0);
  EXPECT_GT(u[2], 0.0);
  EXPECT_GT(u[1], 0.0);
}

TEST_F(PseudoinverseAllocatorTests,
       P09_Pseudo_PositivePitchWrenchInducesPositivePitchMoment) {
  Vector6d tau;
  double fill[6] = {0.0, 0.0, 0.0, 0.0, 100.0, 0.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  // Sanity tests for direction
  EXPECT_LT(u[6], 0.0);
  EXPECT_GT(u[5], 0.0);
  EXPECT_GT(u[2], 0.0);
  EXPECT_LT(u[1], 0.0);
}

TEST_F(PseudoinverseAllocatorTests,
       T10_Pseudo_PositiveYawWrenchInducesPositiveYawMoment) {
  Vector6d tau;
  double fill[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 100.0};

  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  print_input(u);

  EXPECT_GT(u[7], 0.0);
  EXPECT_GT(u[4], 0.0);
  EXPECT_LT(u[3], 0.0);
  EXPECT_LT(u[0], 0.0);
}

// ---------------------------- QP allocator tests ----------------------------

TEST_F(QPAllocatorTests, Q0_QP_ThrowsOnTauDimensionMismatch) {
  Eigen::VectorXd tau_wrong =
      Eigen::VectorXd::Zero(allocator_config.extended_thrust_matrix.rows() + 1);

  EXPECT_THROW(allocator->calculate_allocated_thrust(tau_wrong),
               std::runtime_error);
}

TEST_F(QPAllocatorTests, Q01_QP_ZeroWrenchGivesFeasibleBoundedThrust) {
  Vector6d tau = Vector6d::Zero();
  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);

  EXPECT_NEAR(u.norm(), 0.0, 1e-6);
}

TEST_F(QPAllocatorTests, Q02_QP_OutputRespectsBoundsUnderHugeWrench) {
  Vector6d tau;
  double fill[6] = {100000.0, 100000.0, 100000.0, 100000.0, 100000.0, 100000.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);
}

TEST_F(QPAllocatorTests, Q03_QP_ForwardWrenchGivesForwardThrust) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_GT(u[7], eps);
  EXPECT_GT(u[4], eps);
  EXPECT_GT(u[3], eps);
  EXPECT_GT(u[0], eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q04_QP_BackwardWrenchGivesBackwardThrust) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {-100.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_LT(u[7], -eps);
  EXPECT_LT(u[4], -eps);
  EXPECT_LT(u[3], -eps);
  EXPECT_LT(u[0], -eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q05_QP_SidewaysWrenchGivesSidewaysThrust) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {0.0, 100.0, 0.0, 0.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_GT(u[7], eps);
  EXPECT_LT(u[4], -eps);
  EXPECT_GT(u[3], eps);
  EXPECT_LT(u[0], -eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q06_QP_HeavePlusGivesPositiveVerticalThrust) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {0.0, 0.0, 100.0, 0.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_GT(u[6], eps);
  EXPECT_GT(u[5], eps);
  EXPECT_GT(u[2], eps);
  EXPECT_GT(u[1], eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q07_QP_HeaveMinusGivesNegativeVerticalThrust) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {0.0, 0.0, -100.0, 0.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_LT(u[6], -eps);
  EXPECT_LT(u[5], -eps);
  EXPECT_LT(u[2], -eps);
  EXPECT_LT(u[1], -eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q08_QP_PositiveRollWrenchInducesPositiveRollMoment) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {0.0, 0.0, 0.0, 60.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_LT(u[6], -eps);
  EXPECT_LT(u[5], -eps);
  EXPECT_GT(u[2], eps);
  EXPECT_GT(u[1], eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  std::cout << "tau_hat: " << tau_hat << std::endl;
  std::cout << "tau: " << tau << std::endl;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q09_QP_PositivePitchWrenchInducesPositivePitchMoment) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {0.0, 0.0, 0.0, 0.0, 100.0, 0.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_LT(u[6], -eps);
  EXPECT_GT(u[5], eps);
  EXPECT_GT(u[2], eps);
  EXPECT_LT(u[1], -eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q10_QP_PositiveYawWrenchInducesPositiveYawMoment) {
  const double eps = 1e-6;

  Vector6d tau;
  double fill[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 100.0};
  fill_input_from_double_array(tau, fill);

  Eigen::VectorXd u = require_thrust(allocator, tau);

  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  EXPECT_GT(u[7], eps);
  EXPECT_GT(u[4], eps);
  EXPECT_LT(u[3], -eps);
  EXPECT_LT(u[0], -eps);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

// ---------------------------- Non-trivial testing ----------------------------

TEST_F(QPAllocatorTests, Q11_QP_HeavePlusAndRollPlus_UsesVerticalThrusters) {
  Vector6d tau;
  double fill[6] = {0.0, 0.0, 80.0, 40.0, 0.0, 0.0};
  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  // Active set (vertical): 1,2,5,6
  // Quiet set  (horizontal): 0,3,4,7
  expect_group_dominance(u, {1, 2, 5, 6}, {0, 3, 4, 7});

  const double eps_active = 1; // must be "meaningfully used"

  EXPECT_GT(std::abs(u[1]), eps_active);
  EXPECT_GT(std::abs(u[2]), eps_active);
  EXPECT_GT(std::abs(u[5]), eps_active);
  EXPECT_GT(std::abs(u[6]), eps_active);

  // Compute mean absolute thrust of active (vertical) thrusters and use it as a
  // relative threshold
  double mean_active =
      (std::abs(u[1]) + std::abs(u[2]) + std::abs(u[5]) + std::abs(u[6])) / 4.0;
  double eps_quiet = mean_active * 0.1;

  EXPECT_NEAR(u[0], 0.0, eps_quiet);
  EXPECT_NEAR(u[3], 0.0, eps_quiet);
  EXPECT_NEAR(u[4], 0.0, eps_quiet);
  EXPECT_NEAR(u[7], 0.0, eps_quiet);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

TEST_F(QPAllocatorTests, Q12_QP_SurgePlusAndYawPlus_UsesHorizontalThrusters) {
  Vector6d tau;
  double fill[6] = {80.0, 0.0, 0.0, 0.0, 0.0, 40.0};
  fill_input_from_double_array(tau, fill);
  print_generalized_force(tau);

  Eigen::VectorXd u = require_thrust(allocator, tau);
  expect_valid_qp_output(u, allocator_config);
  print_input(u);

  // Active set (horizontal): 0,3,4,7
  // Quiet set  (vertical):   1,2,5,6
  expect_group_dominance(u, {0, 3, 4, 7}, {1, 2, 5, 6});

  const double eps_active = 1; // must be "meaningfully used"

  EXPECT_GT(std::abs(u[0]), eps_active);
  EXPECT_GT(std::abs(u[3]), eps_active);
  EXPECT_GT(std::abs(u[4]), eps_active);
  EXPECT_GT(std::abs(u[7]), eps_active);

  // Compute mean absolute thrust of active (vertical) thrusters and use it as a
  // relative threshold
  double mean_active =
      (std::abs(u[0]) + std::abs(u[3]) + std::abs(u[4]) + std::abs(u[7])) / 4.0;
  double eps_quiet = mean_active * 0.1;

  EXPECT_NEAR(u[1], 0.0, eps_quiet);
  EXPECT_NEAR(u[2], 0.0, eps_quiet);
  EXPECT_NEAR(u[5], 0.0, eps_quiet);
  EXPECT_NEAR(u[6], 0.0, eps_quiet);

  Eigen::VectorXd tau_hat = allocator_config.extended_thrust_matrix * u;
  EXPECT_NEAR((tau_hat - tau).norm(), 0.0, tau_tol);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

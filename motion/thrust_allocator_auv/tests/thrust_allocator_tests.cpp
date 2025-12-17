#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <vortex/utils/math.hpp>
#include "vortex/utils/types.hpp"
#include "thrust_allocator_auv/allocator.hpp"
#include "thrust_allocator_auv/qp_allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/allocator_factory.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"

using vortex::utils::types::Vector6d;

void print_generalized_force(const Vector6d& tau) {
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

void print_input(const Eigen::VectorXd& thruster_forces) {
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

AllocatorConfig load_allocator_config(const std::string& yaml_path)
{

    YAML::Node params      = YAML::LoadFile(yaml_path)["/**"]["ros__parameters"];
    YAML::Node physical    = params["physical"];
    YAML::Node thrusters   = params["propulsion"]["thrusters"];
    YAML::Node constraints = thrusters["constraints"];

    const int num_thrusters = thrusters["num"].as<int>();

    Eigen::MatrixXd thruster_force_directions =
        double_array_to_eigen_matrix(
            thrusters["thruster_force_direction"].as<std::vector<double>>(),
            3,
            num_thrusters);

    Eigen::MatrixXd thruster_positions =
        double_array_to_eigen_matrix(
            thrusters["thruster_position"].as<std::vector<double>>(),
            3,
            num_thrusters);

    Eigen::Vector3d center_of_mass =
        double_array_to_eigen_vector3d(
            physical["center_of_mass"].as<std::vector<double>>());

    Eigen::MatrixXd thrust_configuration_matrix =
        calculate_thrust_configuration_matrix(
            thruster_force_directions,
            thruster_positions,
            center_of_mass);

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

    Eigen::VectorXd input_weight_diagonal =
        Eigen::Map<const Eigen::VectorXd>(input_weight_values.data(), num_thrusters);

    Eigen::VectorXd slack_weight_diagonal =
        Eigen::Map<const Eigen::VectorXd>(slack_weight_values.data(), num_thrusters);


    AllocatorConfig config;
    config.extended_thrust_matrix = thrust_configuration_matrix;
    config.min_force              = min_force;
    config.max_force              = max_force;
    config.input_weight_matrix    = input_weight_diagonal.asDiagonal();
    config.slack_weight_matrix    = slack_weight_diagonal.asDiagonal();
    config.beta                   = constraints["beta"].as<double>();

    return config;
}

class ThrustAllocatorYamlTests : public ::testing::Test {
protected:
    AllocatorConfig allocator_config;

    void SetUp() override {
        allocator_config = load_allocator_config(YAML_PATH);
    }
};

TEST_F(ThrustAllocatorYamlTests, T00_correctly_pulled_AllocatorConfig_from_yaml_file)
{
    ASSERT_EQ(allocator_config.extended_thrust_matrix.rows(), 6);
    ASSERT_EQ(allocator_config.extended_thrust_matrix.cols(), 8);

    ASSERT_EQ(allocator_config.min_force.size(), allocator_config.max_force.size());
    ASSERT_EQ(allocator_config.extended_thrust_matrix.cols(), allocator_config.min_force.size());

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

TEST_F(PseudoinverseAllocatorTests, ZeroWrenchGivesZeroThrust)
{
    Vector6d tau = Vector6d::Zero();
    Eigen::VectorXd u = allocator->calculate_allocated_thrust(tau);
    EXPECT_NEAR(u.norm(), 0.0, 1e-9);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
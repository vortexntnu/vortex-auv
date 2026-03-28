#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <vortex/utils/types.hpp>

#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

namespace vortex::control {

using vortex::utils::types::PoseEuler;
using vortex::utils::types::Twist;

DPAdaptParams load_dp_adapt_params(const std::string& drone_yaml_path,
                                   const std::string& controller_yaml_path) {
    YAML::Node drone_params =
        YAML::LoadFile(drone_yaml_path)["/**"]["ros__parameters"];
    YAML::Node controller_params =
        YAML::LoadFile(controller_yaml_path)["/**"]["ros__parameters"];

    auto K1_vec = controller_params["K1"].as<std::vector<double>>();
    auto K2_vec = controller_params["K2"].as<std::vector<double>>();
    auto adapt_gain_vec =
        controller_params["adapt_gain"].as<std::vector<double>>();
    auto d_gain_vec = controller_params["d_gain"].as<std::vector<double>>();
    auto r_b_bg_vec = controller_params["r_b_bg"].as<std::vector<double>>();

    auto mass_matrix_vec =
        drone_params["physical"]["mass_matrix"].as<std::vector<double>>();

    Eigen::Matrix6d mass_matrix =
        Eigen::Map<Eigen::Matrix6d>(mass_matrix_vec.data());

    DPAdaptParams params;
    params.K1 = Eigen::Map<Eigen::Vector6d>(K1_vec.data());
    params.K2 = Eigen::Map<Eigen::Vector6d>(K2_vec.data());
    params.adapt_param = Eigen::Map<Eigen::Vector12d>(adapt_gain_vec.data());
    params.d_gain = Eigen::Map<Eigen::Vector6d>(d_gain_vec.data());
    params.r_b_bg = Eigen::Map<Eigen::Vector3d>(r_b_bg_vec.data());
    params.mass_matrix = mass_matrix;
    params.mass = mass_matrix(0, 0);
    params.I_b = Eigen::Vector3d(mass_matrix(3, 3), mass_matrix(4, 4),
                                 mass_matrix(5, 5));
    return params;
}

class DPAdaptBacksControllerTests : public ::testing::Test {
   protected:
    DPAdaptBacksControllerTests()
        : dp_adapt_backs_controller_{
              load_dp_adapt_params(DRONE_YAML_PATH, CONTROLLER_YAML_PATH)} {}

    PoseEuler generate_current_pose(const double north_pos,
                                    const double east_pos,
                                    const double down_pos,
                                    const double roll_angle,
                                    const double pitch_angle,
                                    const double yaw_angle) {
        return {north_pos,  east_pos,    down_pos,
                roll_angle, pitch_angle, yaw_angle};
    }

    PoseEuler generate_reference_pose(const double north_pos,
                                      const double east_pos,
                                      const double down_pos,
                                      const double roll_angle,
                                      const double pitch_angle,
                                      const double yaw_angle) {
        return {north_pos,  east_pos,    down_pos,
                roll_angle, pitch_angle, yaw_angle};
    }

    Twist generate_current_velocity(const double surge_vel,
                                    const double sway_vel,
                                    const double heave_vel,
                                    const double roll_rate,
                                    const double pitch_rate,
                                    const double yaw_rate) {
        return {surge_vel, sway_vel,   heave_vel,
                roll_rate, pitch_rate, yaw_rate};
    }

    DPAdaptBacksController dp_adapt_backs_controller_;
};

/*
Test that negative north error only (in body) gives positive surge command only.
*/

TEST_F(DPAdaptBacksControllerTests,
       T01_neg_north_error_with_zero_heading_gives_surge_only_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_GT(tau[0], 0.0);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative north error with positive heading gives a positive surge
command and negative sway command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T02_neg_north_error_with_positive_heading_gives_pos_surge_and_neg_sway_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    PoseEuler pose_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_GT(tau[0], 0.0);
    EXPECT_LT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative north error with negative heading gives a positive surge
command and positive sway command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T03_neg_north_error_with_negative_heading_gives_pos_surge_and_pos_sway_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    PoseEuler pose_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_GT(tau[0], 0.0);
    EXPECT_GT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative down error with zero roll and pitch gives a positive heave
command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T04_neg_down_error_with_zero_roll_and_pitch_gives_positive_heave_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_GT(tau[2], 0.0);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative down error with zero roll and negative pitch gives a positive
heave and positive surge command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T05_neg_down_error_with_zero_roll_and_neg_pitch_gives_positive_heave_and_positive_surge_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, -0.5, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, -0.5, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_GT(tau[0], 0.0);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_GT(tau[2], 0.0);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative down error with zero roll and positive pitch gives a positive
heave and negative surge command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T06_neg_down_error_with_zero_roll_and_pos_pitch_gives_positive_heave_and_negative_surge_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.5, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, 0.5, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_LT(tau[0], 0.0);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_GT(tau[2], 0.0);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative east error with zero heading gives a positive sway command.
*/

TEST_F(DPAdaptBacksControllerTests,
       T07_neg_east_error_with_zero_heading_gives_positive_sway_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_GT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that positive east error with zero heading gives a negative sway command.
*/

TEST_F(DPAdaptBacksControllerTests,
       T08_pos_east_error_with_zero_heading_gives_pos_sway_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, -10.0, 0.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_LT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative east error with positive heading gives a positive surge and
sway command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T09_neg_east_error_with_positive_heading_gives_pos_sway_and_pos_surge_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    PoseEuler pose_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, 1.5)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_GT(tau[0], 0.0);
    EXPECT_GT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative east error with negative heading gives a negative surge and
positive sway command.
*/

TEST_F(
    DPAdaptBacksControllerTests,
    T10_neg_east_error_with_negative_heading_gives_pos_sway_and_neg_surge_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    PoseEuler pose_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, -1.5)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_LT(tau[0], 0.0);
    EXPECT_GT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative roll error gives positive roll command.
*/

TEST_F(DPAdaptBacksControllerTests,
       T11_neg_roll_error_gives_positive_roll_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_GT(tau[3], 0.0);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that positive roll error gives negative roll command.
*/

TEST_F(DPAdaptBacksControllerTests, T12_pos_roll_error_gives_neg_roll_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, -1.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_LT(tau[3], 0.0);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative pitch error gives positive pitch command.
*/

TEST_F(DPAdaptBacksControllerTests,
       T13_neg_pitch_error_gives_pos_pitch_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_GT(tau[4], 0.0);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that positive pitch error gives negative pitch command.
*/

TEST_F(DPAdaptBacksControllerTests,
       T14_pos_pitch_error_gives_neg_pitch_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, -1.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_LT(tau[4], 0.0);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that negative yaw error gives positive yaw command.
*/

TEST_F(DPAdaptBacksControllerTests, T15_neg_yaw_error_gives_pos_yaw_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_GT(tau[5], 0.0);
}

/*
Test that positive yaw error gives negative yaw command.
*/

TEST_F(DPAdaptBacksControllerTests, T16_pos_yaw_error_gives_neg_yaw_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_LT(tau[5], 0.0);
}

/*
Test that positive surge velocity only results in negative surge command
(breaking effect).
*/

TEST_F(DPAdaptBacksControllerTests,
       T17_pos_surge_vel_gives_negative_surge_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_LT(tau[0], 0.0);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
    // Torque channels may have cross-coupling from off-diagonal mass matrix
    // terms in the Coriolis matrix
}

/*
Test that positive sway velocity only results in negative sway command (breaking
effect).
*/

TEST_F(DPAdaptBacksControllerTests,
       T18_pos_sway_vel_gives_negative_sway_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_LT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
    // Torque channels may have cross-coupling from off-diagonal mass matrix
    // terms in the Coriolis matrix
}

/*
Test that positive heave velocity only results in negative heave command
(breaking effect).
*/

TEST_F(DPAdaptBacksControllerTests,
       T19_pos_heave_vel_gives_negative_heave_command) {
    PoseEuler pose{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    PoseEuler pose_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Twist twist{generate_current_velocity(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(pose, pose_d, twist)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_LT(tau[2], 0.0);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
    // Torque channels may have cross-coupling from off-diagonal mass matrix
    // terms in the Coriolis matrix
}

}  // namespace vortex::control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

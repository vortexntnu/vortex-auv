#include <gtest/gtest.h>

#include <vortex_utils/types.hpp>

#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

namespace vortex::control {

using vortex::utils::types::Eta;
using vortex::utils::types::Nu;

class DPAdaptBacksControllerTests : public ::testing::Test {
   protected:
    DPAdaptBacksControllerTests()
        : dp_adapt_backs_controller_{get_dp_params()} {}

    DPAdaptParams get_dp_params() {
        DPAdaptParams params;
        params.adapt_param = Eigen::Vector12d::Ones() * 0.1;
        params.d_gain = Eigen::Vector6d::Ones() * 0.6;
        params.K1 = Eigen::Vector6d::Ones() * 5.0;
        params.K2 = Eigen::Vector6d::Ones() * 5.0;
        params.r_b_bg = Eigen::Vector3d(0.01, 0.0, 0.02);
        params.I_b = Eigen::Vector3d(0.68, 3.32, 3.34);
        params.mass_matrix = Eigen::Matrix6d::Identity() * 30.0;
        params.mass = 30.0;
        return params;
    }

    Eta generate_current_pose(const double north_pos,
                              const double east_pos,
                              const double down_pos,
                              const double roll_angle,
                              const double pitch_angle,
                              const double yaw_angle) {
        return {north_pos,  east_pos,    down_pos,
                roll_angle, pitch_angle, yaw_angle};
    }

    Eta generate_reference_pose(const double north_pos,
                                const double east_pos,
                                const double down_pos,
                                const double roll_angle,
                                const double pitch_angle,
                                const double yaw_angle) {
        return {north_pos,  east_pos,    down_pos,
                roll_angle, pitch_angle, yaw_angle};
    }

    Nu generate_current_velocity(const double surge_vel,
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    Eta eta_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    Eta eta_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, -0.5, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, -0.5, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.5, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, 0.5, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, -10.0, 0.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    Eta eta_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, 1.5)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    Eta eta_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, -1.5)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, -1.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, -1.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
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
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
    EXPECT_LT(tau[0], 0.0);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that positive sway velocity only results in negative sway command (breaking
effect).
*/

TEST_F(DPAdaptBacksControllerTests,
       T18_pos_sway_vel_gives_negative_sway_command) {
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_LT(tau[1], 0.0);
    EXPECT_NEAR(tau[2], 0.0, 0.01);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

/*
Test that positive heave velocity only results in negative heave command
(breaking effect).
*/

TEST_F(DPAdaptBacksControllerTests,
       T19_pos_heave_vel_gives_negative_heave_command) {
    Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    Nu nu{generate_current_velocity(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)};
    Eigen::Vector6d tau{
        dp_adapt_backs_controller_.calculate_tau(eta, eta_d, nu)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_LT(tau[2], 0.0);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

}  // namespace vortex::control

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

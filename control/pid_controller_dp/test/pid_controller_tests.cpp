#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

void print_tau(const types::Vector6d& tau) {
    spdlog::info("Tau values:");
    spdlog::info("Surge: {}", tau[0]);
    spdlog::info("Sway: {}", tau[1]);
    spdlog::info("Heave: {}", tau[2]);
    spdlog::info("Roll: {}", tau[3]);
    spdlog::info("Pitch: {}", tau[4]);
    spdlog::info("Yaw: {}", tau[5]);
}

class PIDControllerTests : public ::testing::Test {
   protected:
    PIDControllerTests() : pid_controller_() {
        // Set PID gains for testing
        types::Matrix6d Kp = types::Matrix6d::Identity() * 10.0;
        types::Matrix6d Ki = types::Matrix6d::Identity() * 0.5;
        types::Matrix6d Kd = types::Matrix6d::Identity() * 2.0;

        pid_controller_.set_kp(Kp);
        pid_controller_.set_ki(Ki);
        pid_controller_.set_kd(Kd);
    }

    types::Eta generate_current_pose(const double north_pos,
                                     const double east_pos,
                                     const double down_pos,
                                     const double roll_angle,
                                     const double pitch_angle,
                                     const double yaw_angle) {
        types::Eta current_pose;
        current_pose.x = north_pos;
        current_pose.y = east_pos;
        current_pose.z = down_pos;
        Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(
            roll_angle, pitch_angle, yaw_angle);
        current_pose.qw = q.w();
        current_pose.qx = q.x();
        current_pose.qy = q.y();
        current_pose.qz = q.z();
        return current_pose;
    }

    types::Eta generate_reference_pose(const double north_pos,
                                       const double east_pos,
                                       const double down_pos,
                                       const double roll_angle,
                                       const double pitch_angle,
                                       const double yaw_angle) {
        types::Eta reference_pose;
        reference_pose.x = north_pos;
        reference_pose.y = east_pos;
        reference_pose.z = down_pos;
        Eigen::Quaterniond q = vortex::utils::math::euler_to_quat(
            roll_angle, pitch_angle, yaw_angle);
        reference_pose.qw = q.w();
        reference_pose.qx = q.x();
        reference_pose.qy = q.y();
        reference_pose.qz = q.z();
        return reference_pose;
    }

    types::Nu generate_current_velocity(const double surge_vel,
                                        const double sway_vel,
                                        const double heave_vel,
                                        const double roll_rate,
                                        const double pitch_rate,
                                        const double yaw_rate) {
        types::Nu current_velocity;
        current_velocity.u = surge_vel;
        current_velocity.v = sway_vel;
        current_velocity.w = heave_vel;
        current_velocity.p = roll_rate;
        current_velocity.q = pitch_rate;
        current_velocity.r = yaw_rate;
        return current_velocity;
    }

    PIDController pid_controller_;
};

/*
Test that negative north error only (in body) gives positive surge command only.
*/

TEST_F(PIDControllerTests,
       T01_neg_north_error_with_zero_heading_gives_surge_only_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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
    PIDControllerTests,
    T02_neg_north_error_with_positive_heading_gives_pos_surge_and_neg_sway_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    types::Eta eta_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);
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
    PIDControllerTests,
    T03_neg_north_error_with_negative_heading_gives_pos_surge_and_pos_sway_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    types::Eta eta_d{generate_reference_pose(10.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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
    PIDControllerTests,
    T04_neg_down_error_with_zero_roll_and_pitch_gives_positive_heave_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);
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
    PIDControllerTests,
    T05_neg_down_error_with_zero_roll_and_neg_pitch_gives_positive_heave_and_positive_surge_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, -0.5, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, -0.5, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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
    PIDControllerTests,
    T06_neg_down_error_with_zero_roll_and_pos_pitch_gives_positive_heave_and_negative_surge_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.5, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 2.0, 0.0, 0.5, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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

TEST_F(PIDControllerTests,
       T07_neg_east_error_with_zero_heading_gives_positive_sway_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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

TEST_F(PIDControllerTests,
       T08_pos_east_error_with_zero_heading_gives_pos_sway_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, -10.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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
    PIDControllerTests,
    T09_neg_east_error_with_positive_heading_gives_pos_sway_and_pos_surge_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5)};
    types::Eta eta_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, 1.5)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
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
    PIDControllerTests,
    T10_neg_east_error_with_negative_heading_gives_pos_sway_and_neg_surge_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.5)};
    types::Eta eta_d{generate_reference_pose(0.0, 10.0, 0.0, 0.0, 0.0, -1.5)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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

TEST_F(PIDControllerTests, T11_neg_roll_error_gives_positive_roll_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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

TEST_F(PIDControllerTests, T12_pos_roll_error_gives_neg_roll_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, -1.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
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

TEST_F(PIDControllerTests, T13_neg_pitch_error_gives_pos_pitch_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
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

TEST_F(PIDControllerTests, T14_pos_pitch_error_gives_neg_pitch_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, -1.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
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

TEST_F(PIDControllerTests, T15_neg_yaw_error_gives_pos_yaw_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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

TEST_F(PIDControllerTests, T16_pos_yaw_error_gives_neg_yaw_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, -1.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    print_tau(tau);

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

TEST_F(PIDControllerTests, T17_pos_surge_vel_gives_negative_surge_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
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

TEST_F(PIDControllerTests, T18_pos_sway_vel_gives_negative_sway_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 1.0, 0.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
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

TEST_F(PIDControllerTests, T19_pos_heave_vel_gives_negative_heave_command) {
    types::Eta eta{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_d{generate_reference_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
    types::Eta eta_dot_d{generate_current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};

    types::Nu nu{generate_current_velocity(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)};
    types::Vector6d tau{
        pid_controller_.calculate_tau(eta, eta_d, nu, eta_dot_d)};
    EXPECT_NEAR(tau[0], 0.0, 0.01);
    EXPECT_NEAR(tau[1], 0.0, 0.01);
    EXPECT_LT(tau[2], 0.0);
    EXPECT_NEAR(tau[3], 0.0, 0.01);
    EXPECT_NEAR(tau[4], 0.0, 0.01);
    EXPECT_NEAR(tau[5], 0.0, 0.01);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

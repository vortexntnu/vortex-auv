#include <gtest/gtest.h>
#include "velocity_controller/lib/3DOF_PID.hpp"
#include "velocity_controller/tests/controller_test_accessor.hpp"
#include "velocity_controller/utilities.hpp"

namespace {

controller_params make_dummy_controller_params() {
    controller_params p;
    p.num_dimensions = 3;
    p.num_thrusters = 4;
    p.thruster_position.resize(4, 3);
    p.thruster_position <<  0.5,  0.5, 0.0,
                             0.5, -0.5, 0.0,
                            -0.5,  0.5, 0.0,
                            -0.5, -0.5, 0.0;
    p.thruster_force_direction.resize(4, 3);
    p.thruster_force_direction << 1, 0, 0,
                                   1, 0, 0,
                                   1, 0, 0,
                                   1, 0, 0;
    p.center_of_mass = Eigen::Vector3d::Zero();
    p.min_thrust = -100.0;
    p.max_thrust = 100.0;
    return p;
}

PID_3DOF_params make_valid_pid3dof_params(double dt = 0.01) {
    PID_3DOF_params p;
    p.surge = {500.0, 50.0, 5.0};
    p.pitch = {60.0, 8.0, 12.0};
    p.yaw   = {10.0, 1.0, 5.0};
    p.dt = dt;
    return p;
}

}  // namespace

class PID3DOFTest : public ::testing::Test {
protected:
    controller_params cp = make_dummy_controller_params();
    PID_3DOF_params pp = make_valid_pid3dof_params();
};

TEST_F(PID3DOFTest, SurgeOutputSaturatesAtThrusterLimit) {
    PID_3DOF pid(pp, cp);
    double tau_max_surge = ControllerTestAccessor::get_tau_max(pid)[0];

    State state{};
    State error{};
    error.surge = 1000.0;
    auto wrench = pid.calculate_thrust(state, error);

    EXPECT_DOUBLE_EQ(wrench.wrench.force.x, tau_max_surge);
}

TEST_F(PID3DOFTest, SurgeOutputSaturatesAtNegativeThrusterLimit) {
    PID_3DOF pid(pp, cp);
    double tau_max_surge = ControllerTestAccessor::get_tau_max(pid)[0];

    State state{};
    State error{};
    error.surge = -1000.0;
    auto wrench = pid.calculate_thrust(state, error);

    EXPECT_DOUBLE_EQ(wrench.wrench.force.x, -tau_max_surge);
}

TEST_F(PID3DOFTest, PitchOutputSaturatesAtThrusterLimit) {
    PID_3DOF pid(pp, cp);
    double tau_max_pitch = ControllerTestAccessor::get_tau_max(pid)[4];

    State state{};
    State error{};
    error.pitch = 1000.0;
    auto wrench = pid.calculate_thrust(state, error);

    EXPECT_DOUBLE_EQ(wrench.wrench.torque.y, tau_max_pitch);
}

TEST_F(PID3DOFTest, YawOutputSaturatesAtThrusterLimit) {
    PID_3DOF pid(pp, cp);
    double tau_max_yaw = ControllerTestAccessor::get_tau_max(pid)[5];

    State state{};
    State error{};
    error.yaw = 1000.0;
    auto wrench = pid.calculate_thrust(state, error);

    EXPECT_DOUBLE_EQ(wrench.wrench.torque.z, tau_max_yaw);
}
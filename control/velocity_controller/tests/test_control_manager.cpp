#include <gtest/gtest.h>
#include "velocity_controller/control_manager.hpp"
#include "velocity_controller/utilities.hpp"
#include "velocity_controller/tests/controller_test_accessor.hpp"


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

LQR_params make_valid_lqr_params(double interval = 0.01) {
    std::vector<double> inertia(36, 0.0);
    inertia[0] = 30.0;
    inertia[1 * 6 + 1] = 30.0;
    inertia[2 * 6 + 2] = 30.0;
    inertia[3 * 6 + 3] = 2.0;
    inertia[4 * 6 + 4] = 3.0;
    inertia[5 * 6 + 5] = 3.0;

    return LQR_params(
        /*Q=*/ {200.0, 32.84, 32.84, 15.0, 15.0, 100.0, 32.84, 32.84},
        /*R=*/ {0.02, 3.1, 3.10},
        /*inertia_matrix=*/ inertia,
        /*D_low=*/ std::vector<double>{104.0,0,0,0,0,0, 0,46,0,0,0,0, 0,0,46,0,0,0,
                                        0,0,0,46,0,0, 0,0,0,0,46,0, 0,0,0,0,0,46},
        /*D_high=*/ std::vector<double>(36, 1.0),
        interval);
}

control_manager_params make_manager_params(int control_type, bool anti_overshoot) {
    control_manager_params p;
    p.control_type = control_type;
    p.anti_overshoot = anti_overshoot;
    p.fallback = false;
    p.control_params = make_dummy_controller_params();
    return p;
}

}  // namespace

// ---------- Guard mot uinitialisert controller ----------

TEST(ControlManagerGuardTest, GetOutputThrowsIfPIDNotInitialized) {
    control_manager cm(make_manager_params(1, false));
    Guidance_data guidance{};
    State state{};
    EXPECT_THROW(cm.get_output(guidance, state), std::runtime_error);
}

TEST(ControlManagerGuardTest, GetOutputThrowsIfLQRNotInitialized) {
    control_manager cm(make_manager_params(2, false));
    Guidance_data guidance{};
    State state{};
    EXPECT_THROW(cm.get_output(guidance, state), std::runtime_error);
}

TEST(ControlManagerGuardTest, GetValidityThrowsIfNotInitialized) {
    control_manager cm(make_manager_params(1, false));
    EXPECT_THROW(cm.get_validity(), std::runtime_error);
}

TEST(ControlManagerGuardTest, ResetControllersThrowsIfNotInitialized) {
    control_manager cm(make_manager_params(1, false));
    EXPECT_THROW(cm.reset_controllers(), std::runtime_error);
}

TEST(ControlManagerGuardTest, GetOutputWorksAfterInitialization) {
    control_manager cm(make_manager_params(1, false));
    cm.initialize_3DOF_controller(make_valid_pid3dof_params());

    Guidance_data guidance{};
    State state{};
    EXPECT_NO_THROW(cm.get_output(guidance, state));
}

// ---------- Ugyldig control_type: default-gren, ingen guard nødvendig ----------

TEST(ControlManagerTest, InvalidControlTypeReturnsDefaultWrenchWithoutThrowing) {
    control_manager cm(make_manager_params(99, false));
    Guidance_data guidance{};
    State state{};
    geometry_msgs::msg::WrenchStamped wrench;
    EXPECT_NO_THROW(wrench = cm.get_output(guidance, state));
    EXPECT_DOUBLE_EQ(wrench.wrench.force.x, 0.0);
    EXPECT_DOUBLE_EQ(wrench.wrench.torque.y, 0.0);
    EXPECT_DOUBLE_EQ(wrench.wrench.torque.z, 0.0);
}

TEST(ControlManagerTest, InvalidControlTypeGetValidityReturnsFalse) {
    control_manager cm(make_manager_params(99, false));
    EXPECT_FALSE(cm.get_validity());
}

// ---------- Anti-overshoot: grensetest rundt ±pi/2 ----------
// Disse fanger opp den tidligere abs()-relaterte bugen fra control_manager.cpp
// (dobbeltsjekk at fiksen fortsatt gjelder etter eventuelle endringer).

class ControlManagerAntiOvershootTest : public ::testing::Test {
protected:
    control_manager cm{make_manager_params(1, /*anti_overshoot=*/true)};

    void SetUp() override {
        cm.initialize_3DOF_controller(make_valid_pid3dof_params());
    }
};


TEST_F(ControlManagerAntiOvershootTest, SurgeScaledWhenYawWellWithinThreshold) {
    Guidance_data guidance{};
    guidance.surge = 1.0;
    guidance.yaw = 0.5;   // godt innenfor pi/2, guidance.pitch = 0
    State state{};        // identitet - alle vinkler 0

    // Analytisk utledning (current_state er identitet, så
    // error_state_body.yaw = guidance.yaw, error_state_body.pitch = guidance.pitch = 0):
    double expected_error_surge = guidance.surge * std::cos(guidance.yaw) * std::cos(0.0);
    // expected_error_surge = 1.0 * cos(0.5) * 1.0 ≈ 0.8775825618903728

    // Surge-PID: kp=500, ki=50, kd=5, dt=0.01, integral/previous_error starter på 0
    double dt = 0.01;
    double integral = expected_error_surge * dt;
    double derivative_term = 5.0 * (expected_error_surge - 0.0) / dt;
    double raw_output = 500.0 * expected_error_surge + 50.0 * integral + derivative_term;

    // Hent faktisk tau_max[0] for denne fixturen og klem forventningen deretter,
    // slik at testen er korrekt uansett hva geometrien i controller_params gir
    control_manager cm_probe(make_manager_params(1, true));
    cm_probe.initialize_3DOF_controller(make_valid_pid3dof_params());
    // NB: krever at control_manager eksponerer sin interne PID_3DOF for accessoren,
    // ELLER at vi bygger en frittstående PID_3DOF med samme controller_params
    // kun for å lese tau_max - se kommentar under testen.
    PID_3DOF probe_pid(make_valid_pid3dof_params(), make_dummy_controller_params());
    double tau_max_surge = ControllerTestAccessor::get_tau_max(probe_pid)[0];

    double expected_output = std::clamp(raw_output, -tau_max_surge, tau_max_surge);

    auto wrench = cm.get_output(guidance, state);
    EXPECT_NEAR(wrench.wrench.force.x, expected_output, 1e-6);
}

TEST_F(ControlManagerAntiOvershootTest, BoundaryJustBelowPiOverTwo) {
    Guidance_data guidance{};
    guidance.surge = 2.0;
    guidance.yaw = 1.5;  // < pi/2 (1.5708) -> skal trigge skalering
    State state{};
    auto wrench_scaled = cm.get_output(guidance, state);

    cm.reset_controllers();
    guidance.yaw = 1.6;  // > pi/2 -> skal IKKE trigge skalering
    auto wrench_unscaled = cm.get_output(guidance, state);

    // De to skal gi ULIK oppførsel - hvis abs()-bugen er tilbake, vil de
    // uventet gi samme (feil) resultat
    EXPECT_NE(wrench_scaled.wrench.force.x, wrench_unscaled.wrench.force.x);
}

TEST_F(ControlManagerAntiOvershootTest, AntiOvershootDisabledSurgeNeverScaled) {
    control_manager cm_no_overshoot(make_manager_params(1, /*anti_overshoot=*/false));
    cm_no_overshoot.initialize_3DOF_controller(make_valid_pid3dof_params());

    Guidance_data guidance{};
    guidance.surge = 2.0;
    guidance.yaw = 0.1;  // ville trigget skalering hvis anti_overshoot var true
    State state{};
    auto wrench_low_yaw = cm_no_overshoot.get_output(guidance, state);

    cm_no_overshoot.reset_controllers();
    guidance.yaw = 3.0;  // stor yaw - skal fortsatt IKKE skalere siden flagget er av
    auto wrench_high_yaw = cm_no_overshoot.get_output(guidance, state);

    // Uten anti_overshoot skal surge-bidraget være identisk uansett yaw,
    // siden error_state_body.surge alltid er guidance.surge - current_state.surge
    EXPECT_DOUBLE_EQ(wrench_low_yaw.wrench.force.x, wrench_high_yaw.wrench.force.x);
}

// ---------- reset_controllers: delegering med riktig nr ----------

TEST(ControlManagerTest, ResetControllersDelegatesToActiveController) {
    control_manager cm(make_manager_params(1, false));
    cm.initialize_3DOF_controller(make_valid_pid3dof_params());

    Guidance_data guidance{};
    guidance.surge = 5.0;
    State state{};
    cm.get_output(guidance, state);  // bygger opp integral i surge-PID

    EXPECT_NO_THROW(cm.reset_controllers(1));  // skal resette kun surge, ikke krasje
}
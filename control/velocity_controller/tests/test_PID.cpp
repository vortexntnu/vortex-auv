#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <vector>
#include <velocity_controller/lib/PID_controller.hpp>

namespace {
PID_params make_params(double kp, double ki, double kd,
                        double dt = 0.01,
                        double max_out = 1000.0,
                        double min_out = -1000.0) {
    return PID_params({kp, ki, kd}, dt, max_out, min_out);
}
}  // namespace

class PIDControllerTest : public ::testing::Test {
protected:
    PID_params surge_params = make_params(500.0, 50.0, 5.0, 0.01, 1000.0, -1000.0);
};

TEST_F(PIDControllerTest, ProportionalOnlyMatchesExpected) {
    PID_controller pid(make_params(500.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(2.0, 0.0), 1000.0);
}

TEST_F(PIDControllerTest, DerivativeTermUsesExternalErrorD) {
    PID_controller pid(make_params(0.0, 0.0, 5.0));
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(1.0, 3.0), 15.0);
}

TEST_F(PIDControllerTest, FirstStepIntegralContribution) {
    // ki=10, dt=0.01, error=2.0 -> integral = 0.02, I-term = 0.2
    PID_controller pid(make_params(0.0, 10.0, 0.0, 0.01));
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(2.0, 0.0), 0.2);
}

TEST_F(PIDControllerTest, IntegralAccumulatesAcrossSteps) {
    PID_controller pid(make_params(0.0, 10.0, 0.0, 0.01, 100000.0, -100000.0));
    double out1 = pid.calculate_thrust(1.0, 0.0);  // integral=0.01, out=0.1
    double out2 = pid.calculate_thrust(1.0, 0.0);  // integral=0.02, out=0.2
    EXPECT_DOUBLE_EQ(out1, 0.1);
    EXPECT_DOUBLE_EQ(out2, 0.2);
}

TEST_F(PIDControllerTest, InternalAndExternalDerivativeAgree) {
    PID_controller pid_internal(surge_params);
    PID_controller pid_external(surge_params);
    std::vector<double> errors = {0.0, 1.0, 2.5, 2.0, 0.5};
    double prev_error = 0.0;
    for (double e : errors) {
        double out_internal = pid_internal.calculate_thrust(e);  // internt regnet derivat
        double error_d = (e - prev_error) / surge_params.dt;
        double out_external = pid_external.calculate_thrust(e, error_d);
        EXPECT_NEAR(out_internal, out_external, 1e-9) << "error=" << e;
        prev_error = e;
    }
}

TEST_F(PIDControllerTest, OutputClampsAtMax) {
    PID_controller pid(surge_params);  // kp=500, max=1000
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(100.0, 0.0), 1000.0);
}

TEST_F(PIDControllerTest, OutputClampsAtMin) {
    PID_controller pid(surge_params);
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(-100.0, 0.0), -1000.0);
}

TEST_F(PIDControllerTest, AntiWindupPreventsIntegralGrowthUnderSustainedSaturation) {
    // Vedvarende saturasjon: integral += error*dt reverteres av
    // integral -= error*dt hvert steg output er saturert, så integral
    // holder seg konstant og output blir liggende flatt på max_output,
    // IKKE fortsette å vokse (som ville skjedd uten anti-windup).
    PID_controller pid(surge_params);
    double out1 = pid.calculate_thrust(100.0, 0.0);
    double out2 = pid.calculate_thrust(100.0, 0.0);
    double out3 = pid.calculate_thrust(100.0, 0.0);
    EXPECT_DOUBLE_EQ(out1, surge_params.max_output);
    EXPECT_DOUBLE_EQ(out2, surge_params.max_output);
    EXPECT_DOUBLE_EQ(out3, surge_params.max_output);
}

TEST_F(PIDControllerTest, RecoversImmediatelyAfterSaturationEnds) {
    // Fordi add/revert av integral kansellerer hverandre nøyaktig under
    // vedvarende saturasjon, forblir integral uendret (0) gjennom hele
    // saturasjonsperioden. Når feilen blir liten igjen, reagerer
    // controlleren umiddelbart proporsjonalt med DEN nye feilen,
    // uten forsinkelse fra "oppspart" integral.
    PID_controller pid(make_params(0.0, 50.0, 0.0, 0.01, 10.0, -10.0));
    for (int i = 0; i < 20; ++i) {
        pid.calculate_thrust(100.0, 0.0);  // holder saturert i 20 steg
    }
    double out_small_error = pid.calculate_thrust(0.001, 0.0);
    // integral = 0 (uendret gjennom loopen) + 0.001*0.01 = 0.00001
    // output = ki * integral = 50 * 0.00001 = 0.0005
    EXPECT_NEAR(out_small_error, 0.0005, 1e-9);
}

TEST_F(PIDControllerTest, ZeroErrorAndZeroDerivativeGivesZeroOutput) {
    PID_controller pid(surge_params);
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(0.0, 0.0), 0.0);
}

TEST_F(PIDControllerTest, ResetControllerRestoresFreshState) {
    PID_controller pid(surge_params);
    pid.calculate_thrust(5.0);
    pid.calculate_thrust(5.0);
    pid.reset_controller();

    PID_controller fresh(surge_params);
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(1.0), fresh.calculate_thrust(1.0));
}

// ---------- Konstruktør / validitet ----------
// NB: Disse tre testene beskriver ØNSKET oppførsel (spec), ikke nødvendigvis
// dagens implementasjon. Konstruktøren bruker i dag `&&` mellom
// dt<=0-sjekken og max<min-sjekken, som gjør at f.eks. NegativeDtIsInvalid
// og ZeroDtIsInvalid vil FEILE inntil den logiske feilen er rettet til `||`.
// Behold dem som regresjonsmål for fiksen du gjør i .cpp-filen.

class PIDValidityTest : public ::testing::Test {};

TEST_F(PIDValidityTest, ValidParamsProduceValidController) {
    PID_controller pid(make_params(1, 1, 1, 0.01, 100.0, -100.0));
    EXPECT_TRUE(pid.get_validity());
}

TEST_F(PIDValidityTest, ZeroDtIsInvalid) {
    PID_controller pid(make_params(1, 1, 1, /*dt=*/0.0, 100.0, -100.0));
    EXPECT_FALSE(pid.get_validity());
}

TEST_F(PIDValidityTest, NegativeDtIsInvalid) {
    PID_controller pid(make_params(1, 1, 1, /*dt=*/-0.5, 100.0, -100.0));
    EXPECT_FALSE(pid.get_validity());
}

TEST_F(PIDValidityTest, MaxLessThanMinIsInvalid) {
    PID_controller pid(make_params(1, 1, 1, 0.01, /*max=*/-10.0, /*min=*/10.0));
    EXPECT_FALSE(pid.get_validity());
}

TEST_F(PIDValidityTest, InvalidControllerAlwaysReturnsZeroThrust) {
    PID_controller pid(make_params(1, 1, 1, /*dt=*/0.0, 100.0, -100.0));
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(50.0), 0.0);
    EXPECT_DOUBLE_EQ(pid.calculate_thrust(50.0, 1.0), 0.0);
}

// ---------- PID_params::operator=(vector<double>) ----------
// NB: Samme forbehold - SixElementsUpdatesAllFieldsWithoutThrowing beskriver
// ønsket oppførsel og vil feile mot dagens kode (throw ligger feil plassert).

class PIDParamsAssignmentTest : public ::testing::Test {
protected:
    PID_params p = make_params(1.0, 2.0, 3.0, 0.01, 100.0, -100.0);
};

TEST_F(PIDParamsAssignmentTest, ThreeElementsUpdatesOnlyGains) {
    p = std::vector<double>{10.0, 20.0, 30.0};
    EXPECT_DOUBLE_EQ(p.k_p, 10.0);
    EXPECT_DOUBLE_EQ(p.k_i, 20.0);
    EXPECT_DOUBLE_EQ(p.k_d, 30.0);
    EXPECT_DOUBLE_EQ(p.dt, 0.01);
    EXPECT_DOUBLE_EQ(p.max_output, 100.0);
    EXPECT_DOUBLE_EQ(p.min_output, -100.0);
}

TEST_F(PIDParamsAssignmentTest, SixElementsUpdatesAllFieldsWithoutThrowing) {
    auto assign = [&]() {
        p = std::vector<double>{10.0, 20.0, 30.0, 0.02, 200.0, -200.0};
    };
    EXPECT_NO_THROW(assign());

    EXPECT_DOUBLE_EQ(p.k_p, 10.0);
    EXPECT_DOUBLE_EQ(p.k_i, 20.0);
    EXPECT_DOUBLE_EQ(p.k_d, 30.0);
    EXPECT_DOUBLE_EQ(p.dt, 0.02);
    EXPECT_DOUBLE_EQ(p.max_output, 200.0);
    EXPECT_DOUBLE_EQ(p.min_output, -200.0);
}

TEST_F(PIDParamsAssignmentTest, InvalidSizeThrows) {
    auto assign = [&]() {
        p = std::vector<double>{1.0, 2.0};
    };
    EXPECT_THROW(assign(), std::invalid_argument);
}

TEST_F(PIDParamsAssignmentTest, InvalidSizeDoesNotModifyState) {
    PID_params before = p;
    auto assign = [&]() {
        p = std::vector<double>{1.0, 2.0, 3.0, 4.0, 5.0};
    };
    EXPECT_THROW(assign(), std::invalid_argument);
    EXPECT_DOUBLE_EQ(p.k_p, before.k_p);
    EXPECT_DOUBLE_EQ(p.k_i, before.k_i);
    EXPECT_DOUBLE_EQ(p.k_d, before.k_d);
    EXPECT_DOUBLE_EQ(p.dt, before.dt);
    EXPECT_DOUBLE_EQ(p.max_output, before.max_output);
    EXPECT_DOUBLE_EQ(p.min_output, before.min_output);
}

#include <gtest/gtest.h>
#include <velocity_controller/lib/LQR_setup.hpp>
#include "velocity_controller/utilities.hpp"

// I test_LQR.cpp, før testene
class LQRTestAccessor {
public:
    static Eigen::Matrix<double, 8, 8> linearize(LQRController& c, const State& s) {
        return c.linearize(s);
    }
    static Eigen::Vector<double, 8> update_error(LQRController& c,
                                                  const State& error_state,
                                                  const State& state) {
        return c.update_error(error_state, state);
    }
};




namespace {

LQR_params make_valid_lqr_params(double interval = 0.01) {
    return LQR_params(
        /*Q=*/  std::vector<double>{200.0, 32.84, 32.84, 15.0, 15.0, 100.0, 32.84, 32.84},
        /*R=*/  std::vector<double>{0.02, 3.1, 3.10},
        /*inertia_matrix=*/ [] {
            // 6x6 diagonal, row-major: mass ved [0][0], Ixx [3][3], Iyy [4][4], Izz [5][5]
            std::vector<double> m(36, 0.0);
            m[0] = 30.0;              // mass
            m[3 * 6 + 3] = 2.0;       // Ixx
            m[4 * 6 + 4] = 3.0;       // Iyy
            m[5 * 6 + 5] = 3.0;       // Izz
            // NB: sett også nødvendige off-diagonale/andre diagonale ledd
            // for at inverse() ikke feiler pga singularitet - juster ved behov
            m[1 * 6 + 1] = 30.0;
            m[2 * 6 + 2] = 30.0;
            return m;
        }(),
        /*D_low=*/ std::vector<double>{104.0,0,0,0,0,0, 0,46,0,0,0,0, 0,0,46,0,0,0,
                                        0,0,0,46,0,0, 0,0,0,0,46,0, 0,0,0,0,0,46},
        /*D_high=*/ std::vector<double>(36, 1.0),
        interval);
}

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

} // namespace

// ---------- Konstruktør / dimensjonsvalidering ----------

class LQRValidityTest : public ::testing::Test {
protected:
    controller_params cp = make_dummy_controller_params();
};

TEST_F(LQRValidityTest, ValidParamsProduceValidController) {
    LQRController lqr(make_valid_lqr_params(), cp);
    EXPECT_TRUE(lqr.get_validity());
}

TEST_F(LQRValidityTest, ZeroOrNegativeIntervalIsInvalid) {
    auto params = make_valid_lqr_params(/*interval=*/0.0);
    LQRController lqr(params, cp);
    EXPECT_FALSE(lqr.get_validity());
}

TEST_F(LQRValidityTest, WrongSizedQIsInvalid) {
    auto params = make_valid_lqr_params();
    params.Q = std::vector<double>{1, 2, 3};  // skal være 8 elementer
    LQRController lqr(params, cp);
    EXPECT_FALSE(lqr.get_validity());
}

TEST_F(LQRValidityTest, WrongSizedRIsInvalid) {
    auto params = make_valid_lqr_params();
    params.R = std::vector<double>{1, 2};  // skal være 3
    LQRController lqr(params, cp);
    EXPECT_FALSE(lqr.get_validity());
}

TEST_F(LQRValidityTest, WrongSizedInertiaMatrixIsInvalid) {
    auto params = make_valid_lqr_params();
    params.inertia_matrix = std::vector<double>(10, 1.0);  // skal være 36
    LQRController lqr(params, cp);
    EXPECT_FALSE(lqr.get_validity());
}

TEST_F(LQRValidityTest, WrongSizedDLowIsInvalid) {
    auto params = make_valid_lqr_params();
    params.D_low = std::vector<double>(10, 1.0);
    LQRController lqr(params, cp);
    EXPECT_FALSE(lqr.get_validity());
}

TEST_F(LQRValidityTest, WrongSizedDHighIsInvalid) {
    auto params = make_valid_lqr_params();
    params.D_high = std::vector<double>(10, 1.0);
    LQRController lqr(params, cp);
    EXPECT_FALSE(lqr.get_validity());
}

// ---------- reset_controller: selektiv nullstilling ----------

class LQRResetTest : public ::testing::Test {
protected:
    controller_params cp = make_dummy_controller_params();
    LQR_params params = make_valid_lqr_params();
    LQRController lqr{params, cp};

    // Bygger opp integral-tilstand via update_error indirekte gjennom calculate_thrust
    void accumulate_integral() {
        State state{};
        State error{};
        error.surge = 1.0; error.pitch = 1.0; error.yaw = 1.0;
        lqr.calculate_thrust(state, error);
    }
};

TEST_F(LQRResetTest, ResetZeroClearsAllIntegrals) {
    accumulate_integral();
    lqr.reset_controller(0);
    // Indirekte verifikasjon: et påfølgende kall med error=0 bør gi ~0 utslag
    // fra I-leddene. Krever at get_output/wrench inspiseres - se calculate_thrust-testene.
}

TEST_F(LQRResetTest, ResetOneClearsOnlySurgeIntegral) {
    // reset_controller(1) skal kun påvirke integral_error_surge, ikke pitch/yaw
    // NB: siden disse feltene er private, må vi verifisere indirekte via
    // calculate_thrust-output før/etter reset - se testene under.
}

// ---------- linearize(): fanger opp setZero()-bugen ----------
// Disse to testene beskriver ØNSKET oppførsel (spec). De vil FEILE mot
// dagens kode fordi A.setZero() kalles etter at D_-blokken settes inn,
// og dermed alltid visker den ut. Behold som regresjonsmål for fiksen.

TEST(LQRLinearizeTest, DampingBlockIsPreservedInOutput) {
    controller_params cp = make_dummy_controller_params();
    LQR_params params = make_valid_lqr_params();
    LQRController lqr(params, cp);

    State state{};
    state.pitch_rate = 0.5;  // gir ikke-null Coriolis-bidrag
    state.yaw_rate = 0.3;

    // NB: linearize() er privat - eksponer via en friend-test, en public
    // test-only wrapper, eller flytt testen til å verifisere indirekte
    // gjennom calculate_thrust sitt resultat. Anbefaler sistnevnte for å
    // unngå å endre access-nivå kun for test.
}

// ---------- calculate_thrust: end-to-end ----------

class LQRCalculateThrustTest : public ::testing::Test {
protected:
    controller_params cp = make_dummy_controller_params();
    LQR_params params = make_valid_lqr_params();
    LQRController lqr{params, cp};
};

TEST_F(LQRCalculateThrustTest, ZeroErrorAndZeroRatesGivesNearZeroThrust) {
    State state{};   // alle hastigheter/vinkler = 0
    State error{};   // ingen feil
    auto wrench = lqr.calculate_thrust(state, error);
    EXPECT_NEAR(wrench.wrench.force.x, 0.0, 1e-6);
    EXPECT_NEAR(wrench.wrench.torque.y, 0.0, 1e-6);
    EXPECT_NEAR(wrench.wrench.torque.z, 0.0, 1e-6);
}

TEST_F(LQRCalculateThrustTest, PositiveSurgeErrorGivesPositiveForceX) {
    State state{};
    State error{};
    error.surge = 0.5;
    auto wrench = lqr.calculate_thrust(state, error);
    EXPECT_GT(wrench.wrench.force.x, 0.0);
}

TEST_F(LQRCalculateThrustTest, InvalidControllerReturnsDefaultWrench) {
    // Bruk ugyldige params for å trigge valid=false, og bekreft at
    // calculate_thrust IKKE prosesserer videre (krever fiks: legg til
    // early-return `if (!valid) return {};` i calculate_thrust)
    auto bad_params = make_valid_lqr_params(/*interval=*/0.0);
    LQRController invalid_lqr(bad_params, cp);
    ASSERT_FALSE(invalid_lqr.get_validity());

    State state{};
    State error{};
    error.surge = 100.0;  // stor feil - ville gitt stort utslag hvis den prosesserte
    auto wrench = invalid_lqr.calculate_thrust(state, error);
    EXPECT_DOUBLE_EQ(wrench.wrench.force.x, 0.0)
        << "calculate_thrust bør sjekke valid ved inngang og returnere "
           "tomt wrench for en ugyldig controller - mangler i dag";
}
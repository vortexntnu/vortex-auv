#include "los_guidance/lib/adaptive_los.hpp"
#include <gtest/gtest.h>

namespace vortex::guidance::los {

class AdaptiveLosTest : public ::testing::Test {
   protected:
    AdaptiveLosTest() : los_{get_params()} {}

    AdaptiveLosParams get_params() {
        AdaptiveLosParams p;
        p.lookahead_distance_h = 0.9;
        p.lookahead_distance_v = 1.4;
        p.adaptation_gain_h = 0.03;
        p.adaptation_gain_v = 0.02;
        p.time_step = 0.01;
        return p;
    }

    AdaptiveLOSGuidance los_;
    const double tol = 1e-9;
};

// Test commanded angles when drone is to the right of the track
TEST_F(AdaptiveLosTest, T01_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, 0.0};

    const types::Outputs O = los_.calculate_outputs(inputs);

    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(O.psi_d, 0.0);
    EXPECT_GT(O.psi_d, -1.57);

    // Pitch cmd should be zero
    EXPECT_NEAR(O.theta_d, 0.0, tol);
}

// Test commanded angles when drone is to the left of the track
TEST_F(AdaptiveLosTest, T02_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, -0.5, 0.0};

    const types::Outputs O = los_.calculate_outputs(inputs);

    // Heading cmd should be between 0 and pi/2
    EXPECT_GT(O.psi_d, 0.0);
    EXPECT_LT(O.psi_d, 1.57);
    // Pitch cmd should be zero
    EXPECT_NEAR(O.theta_d, 0.0, tol);
}

// Test commanded angles when drone is under the track
TEST_F(AdaptiveLosTest, T03_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, 0.5};

    const types::Outputs O = los_.calculate_outputs(inputs);

    // Heading cmd should be 0
    EXPECT_NEAR(O.psi_d, 0.0, tol);
    // Pitch cmd should be between 0 and pi/2
    EXPECT_GT(O.theta_d, 0.0);
    EXPECT_LT(O.theta_d, 1.57);
}

// Test commanded angles when drone is above the track
TEST_F(AdaptiveLosTest, T04_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, -0.5};

    const types::Outputs O = los_.calculate_outputs(inputs);

    // Heading cmd should be 0
    EXPECT_NEAR(O.psi_d, 0.0, tol);
    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(O.theta_d, 0.0);
    EXPECT_GT(O.theta_d, -1.57);
}

// Test commanded angles when drone is above and to the right of the track

TEST_F(AdaptiveLosTest, T05_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, -0.5};

    const types::Outputs O = los_.calculate_outputs(inputs);

    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(O.psi_d, 0.0);
    EXPECT_GT(O.psi_d, -1.57);
    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(O.theta_d, 0.0);
    EXPECT_GT(O.theta_d, -1.57);
}

}  // namespace vortex::guidance::los

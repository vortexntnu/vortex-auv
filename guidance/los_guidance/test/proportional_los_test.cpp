#include "los_guidance/lib/proportional_los.hpp"
#include <gtest/gtest.h>

namespace vortex::guidance::los {

class ProportionalLosTest : public ::testing::Test {
   protected:
    ProportionalLosTest() : Plos_{get_params()} {}

    ProportionalLosParams get_params() {
        ProportionalLosParams params;
        params.lookahead_distance_h = 10.0;
        params.lookahead_distance_v = 10.0;
        return params;
    }

    ProportionalLOSGuidance Plos_;
    const double tol = 1e-9;
};

// Test commanded angles when drone is to the right of the track
TEST_F(ProportionalLosTest, T06_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, 0.0};

    const types::Outputs O = Plos_.calculate_outputs(inputs);

    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(O.psi_d, 0.0);
    EXPECT_GT(O.psi_d, -1.57);

    // Pitch cmd should be zero
    EXPECT_NEAR(O.theta_d, 0.0, tol);
}

// Test commanded angles when drone is to the left of the track
TEST_F(ProportionalLosTest, T07_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, -0.5, 0.0};

    const types::Outputs O = Plos_.calculate_outputs(inputs);

    // Heading cmd should be between 0 and pi/2
    EXPECT_GT(O.psi_d, 0.0);
    EXPECT_LT(O.psi_d, 1.57);
    // Pitch cmd should be zero
    EXPECT_NEAR(O.theta_d, 0.0, tol);
}

// Test commanded angles when drone is under the track
TEST_F(ProportionalLosTest, T08_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, 0.5};

    const types::Outputs O = Plos_.calculate_outputs(inputs);

    // Heading cmd should be 0
    EXPECT_NEAR(O.psi_d, 0.0, tol);
    // Pitch cmd should be between 0 and pi/2
    EXPECT_GT(O.theta_d, 0.0);
    EXPECT_LT(O.theta_d, 1.57);
}

// Test commanded angles when drone is over the track
TEST_F(ProportionalLosTest, T09_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, -0.5};

    const types::Outputs O = Plos_.calculate_outputs(inputs);

    // Heading cmd should be 0
    EXPECT_NEAR(O.psi_d, 0.0, tol);
    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(O.theta_d, 0.0);
    EXPECT_GT(O.theta_d, -1.57);
}

// Test commanded angles when drone is over and to the right of the track

TEST_F(ProportionalLosTest, T10_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, -0.5};

    const types::Outputs O = Plos_.calculate_outputs(inputs);

    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(O.psi_d, 0.0);
    EXPECT_GT(O.psi_d, -1.57);
    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(O.theta_d, 0.0);
    EXPECT_GT(O.theta_d, -1.57);
}

}  // namespace vortex::guidance::los

#include "los_guidance/lib/adaptive_los.hpp"
#include <gtest/gtest.h>

namespace vortex::guidance::los {  // new namespace added los

class AdaptiveLosTest : public ::testing::Test {
   protected:
    AdaptiveLosTest() : los_{get_params()} {}

    AdaptiveLosParams get_params() {
        AdaptiveLosParams p;
        p.lookahead_distance_h = 1.0;
        p.lookahead_distance_v = 1.0;
        p.gamma_h = 1.0;
        p.gamma_v = 1.0;
        p.time_step = 0.01;
        return p;
    }

    AdaptiveLOSGuidance los_;
    const double tol = 1e-9;
};
/*
TEST_F(AdaptiveLosTest, T01_test_cross_track_error_on_track){
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, 0.0};

    const types::Output O = los_.calculate_outputs(inputs);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.0, tol);
    EXPECT_NEAR(e.z_e, 0.0, tol);
}

TEST_F(AdaptiveLosTests, T02_test_cross_track_error_right_off_track) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, 0.0};

    const types::Output O = los_.calculate_outputs(inputs);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.5, tol);
    EXPECT_NEAR(e.z_e, 0.0, tol);
}

TEST_F(AdaptiveLosTests, T03_test_cross_track_error_left_off_track) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, -0.5, 0.0};

    const types::Output O = los_.calculate_outputs(inputs);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, -0.5, tol);
    EXPECT_NEAR(e.z_e, 0.0, tol);
}

TEST_F(AdaptiveLosTests, T04_test_cross_track_error_under_track) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, 0.5};

    const types::Output O = los_.calculate_outputs(inputs);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.0, tol);
    EXPECT_NEAR(e.z_e, 0.5, tol);
}

TEST_F(AdaptiveLosTests, T05_test_cross_track_error_over_track) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, -0.5};

    const types::Output O = los_.calculate_outputs(inputs);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.0, tol);
    EXPECT_NEAR(e.z_e, -0.5, tol);
}
*/

// Test commanded angles when drone is to the right of the track
TEST_F(AdaptiveLosTest, T06_test_commanded_angles) {
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
TEST_F(AdaptiveLosTest, T07_test_commanded_angles) {
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
TEST_F(AdaptiveLosTest, T08_test_commanded_angles) {
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

// Test commanded angles when drone is over the track
TEST_F(AdaptiveLosTest, T09_test_commanded_angles) {
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

// Test commanded angles when drone is over and to the right of the track

TEST_F(AdaptiveLosTest, T10_test_commanded_angles) {
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

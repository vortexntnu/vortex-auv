#include "los_guidance/lib/proportional_los.hpp"
#include "los_guidance/lib/vector_field_los.hpp"
#include <gtest/gtest.h>

namespace vortex::guidance::los {

class VectorFieldLosTest : public ::testing::Test {
   protected:
    VectorFieldLosTest() : Vflos_{get_params()} {}

    VectorFieldLosParams get_params() {
        VectorFieldLosParams params;
        params.max_approach_angle_h = 30.0 * M_PI / 180.0;  // 30 degrees in rad
        params.max_approach_angle_v = 20.0 * M_PI / 180.0;  // 20 degrees in rad
        params.k_p_h = 0.1;  // needs tuning
        params.k_p_v = 0.1;  // needs tuning
        params.time_step = 0.01;
        return params;
    }

    VectorFieldLOSGuidance Vflos_;
    const double tol = 1e-9;
};

// Test commanded angles when drone is to the right of the track
TEST_F(VectorFieldLosTest, T06_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, 0.0};

    const types::Outputs O = Vflos_.calculate_outputs(inputs);

    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(O.psi_d, 0.0);
    EXPECT_GT(O.psi_d, -1.57);

    // Pitch cmd should be zero
    EXPECT_NEAR(O.theta_d, 0.0, tol);
}

// Test commanded angles when drone is to the left of the track
TEST_F(VectorFieldLosTest, T07_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, -0.5, 0.0};

    const types::Outputs O = Vflos_.calculate_outputs(inputs);

    // Heading cmd should be between 0 and pi/2
    EXPECT_GT(O.psi_d, 0.0);
    EXPECT_LT(O.psi_d, 1.57);

    // Pitch cmd should be zero
    EXPECT_NEAR(O.theta_d, 0.0, tol);
}

// Test commanded angles when drone is under the track
TEST_F(VectorFieldLosTest, T08_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, 0.5};

    const types::Outputs O = Vflos_.calculate_outputs(inputs);

    // Heading cmd should be 0
    EXPECT_NEAR(O.psi_d, 0.0, tol);

    // Pitch cmd should be between 0 and pi/2
    EXPECT_GT(O.theta_d, 0.0);
    EXPECT_LT(O.theta_d, 1.57);
}

// Test commanded angles when drone is over the track
TEST_F(VectorFieldLosTest, T09_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.0, -0.5};

    const types::Outputs O = Vflos_.calculate_outputs(inputs);

    // Heading cmd should be 0
    EXPECT_NEAR(O.psi_d, 0.0, tol);

    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(O.theta_d, 0.0);
    EXPECT_GT(O.theta_d, -1.57);
}

// Test commanded angles when drone is over and to the right of the track
TEST_F(VectorFieldLosTest, T10_test_commanded_angles) {
    types::Inputs inputs;
    inputs.prev_point = types::Point{0.0, 0.0, 0.0};
    inputs.next_point = types::Point{1.0, 0.0, 0.0};
    inputs.current_position = types::Point{0.0, 0.5, -0.5};

    const types::Outputs O = Vflos_.calculate_outputs(inputs);

    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(O.psi_d, 0.0);
    EXPECT_GT(O.psi_d, -1.57);

    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(O.theta_d, 0.0);
    EXPECT_GT(O.theta_d, -1.57);
}

}  // namespace vortex::guidance::los

#include <gtest/gtest.h>
#include "los_guidance/lib/adaptive_los.hpp"
#include "los_guidance/lib/integral_los.hpp"

namespace vortex::guidance::los{ 

    class IntegralLosTest : public ::testing::Test {
    protected:
        IntegralLosTest() : Ilos_{get_params()} {}

        IntegralLosParams get_params() {
            IntegralLosParams params;
            params.lookahead_distance_h = 1.0;
            params.lookahead_distance_v = 1.0;
            params.k_i_h = 0.1; // needs tuning
            params.k_i_v = 0.1; // needs tuning
            params.k_p_h = 0.667; // needs tuning
            params.k_p_v = 0.582; // needs tuning
            params.time_step = 0.01;
            return params;
        }
 
        IntegralLOSGuidance Ilos_;
        const double tol = 1e-9;
    }; 

    // Test commanded angles when drone is to the right of the track
    TEST_F(IntegralLosTest, T06_test_commanded_angles) {
        types::Inputs inputs;
        inputs.prev_point = types::Point{0.0, 0.0, 0.0};
        inputs.next_point = types::Point{1.0, 0.0, 0.0};
        inputs.current_position = types::Point{0.0, 0.5, 0.0};

        const types::Outputs O = Ilos_.calculate_outputs(inputs);

        // Heading cmd should be between -pi/2 and 0
        EXPECT_LT(O.psi_d, 0.0);
        EXPECT_GT(O.psi_d, -1.57);

        // Pitch cmd should be zero
        EXPECT_NEAR(O.theta_d, 0.0, tol);
    }

    // Test commanded angles when drone is to the left of the track
    TEST_F(IntegralLosTest, T07_test_commanded_angles) {
        types::Inputs inputs;
        inputs.prev_point = types::Point{0.0, 0.0, 0.0};
        inputs.next_point = types::Point{1.0, 0.0, 0.0};
        inputs.current_position = types::Point{0.0, -0.5, 0.0};

        const types::Outputs O = Ilos_.calculate_outputs(inputs);

        // Heading cmd should be between 0 and pi/2
        EXPECT_GT(O.psi_d, 0.0);
        EXPECT_LT(O.psi_d, 1.57);
        // Pitch cmd should be zero
        EXPECT_NEAR(O.theta_d, 0.0, tol);
    }

    // Test commanded angles when drone is under the track
    TEST_F(IntegralLosTest, T08_test_commanded_angles) {
        types::Inputs inputs;
        inputs.prev_point = types::Point{0.0, 0.0, 0.0};
        inputs.next_point = types::Point{1.0, 0.0, 0.0};
        inputs.current_position = types::Point{0.0, 0.0, 0.5};

        const types::Outputs O = Ilos_.calculate_outputs(inputs);

        // Heading cmd should be 0
        EXPECT_NEAR(O.psi_d, 0.0, tol);
        // Pitch cmd should be between 0 and pi/2
        EXPECT_GT(O.theta_d, 0.0);
        EXPECT_LT(O.theta_d, 1.57);
    }

    // Test commanded angles when drone is over the track
    TEST_F(IntegralLosTest, T09_test_commanded_angles) {
        types::Inputs inputs;
        inputs.prev_point = types::Point{0.0, 0.0, 0.0};
        inputs.next_point = types::Point{1.0, 0.0, 0.0};
        inputs.current_position = types::Point{0.0, 0.0, -0.5};

        const types::Outputs O = Ilos_.calculate_outputs(inputs);

        // Heading cmd should be 0
        EXPECT_NEAR(O.psi_d, 0.0, tol);
        // Pitch cmd should be between -pi/2 and 0
        EXPECT_LT(O.theta_d, 0.0);
        EXPECT_GT(O.theta_d, -1.57);
    }

    // Test commanded angles when drone is over and to the right of the track

    TEST_F(IntegralLosTest, T10_test_commanded_angles) {
        types::Inputs inputs;
        inputs.prev_point = types::Point{0.0, 0.0, 0.0};
        inputs.next_point = types::Point{1.0, 0.0, 0.0};
        inputs.current_position = types::Point{0.0, 0.5, -0.5};

        const types::Outputs O = Ilos_.calculate_outputs(inputs);

        // Heading cmd should be between -pi/2 and 0
        EXPECT_LT(O.psi_d, 0.0);
        EXPECT_GT(O.psi_d, -1.57);
        // Pitch cmd should be between -pi/2 and 0
        EXPECT_LT(O.theta_d, 0.0);
        EXPECT_GT(O.theta_d, -1.57);
    }

}  // namespace vortex::guidance

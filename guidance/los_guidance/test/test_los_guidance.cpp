#include <gtest/gtest.h>

#include "los_guidance/lib/adaptive_los.hpp"

namespace vortex::guidance {

class LOSTests : public ::testing::Test {
   protected:
    LOSTests() : los_guidance_{get_los_params()} {}

    LOS::Params get_los_params() {
        LOS::Params params;
        params.lookahead_distance_h = 1.0;
        params.lookahead_distance_v = 1.0;
        params.gamma_h = 1.0;
        params.gamma_v = 1.0;
        params.time_step = 0.01;
        return params;
    }

    AdaptiveLOSGuidance los_guidance_;
    const double tol = 1e-9;
};

TEST_F(LOSTests, T01_test_cross_track_error_on_track) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.0, 0.0};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.0, tol);
    EXPECT_NEAR(e.z_e, 0.0, tol);
}

TEST_F(LOSTests, T02_test_cross_track_error_right_off_track) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.5, 0.0};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.5, tol);
    EXPECT_NEAR(e.z_e, 0.0, tol);
}

TEST_F(LOSTests, T03_test_cross_track_error_left_off_track) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, -0.5, 0.0};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, -0.5, tol);
    EXPECT_NEAR(e.z_e, 0.0, tol);
}

TEST_F(LOSTests, T04_test_cross_track_error_under_track) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.0, 0.5};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.0, tol);
    EXPECT_NEAR(e.z_e, 0.5, tol);
}

TEST_F(LOSTests, T05_test_cross_track_error_over_track) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.0, -0.5};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);

    EXPECT_NEAR(e.x_e, 0.0, tol);
    EXPECT_NEAR(e.y_e, 0.0, tol);
    EXPECT_NEAR(e.z_e, -0.5, tol);
}

// Test commanded angles when drone is to the right of the track
TEST_F(LOSTests, T06_test_commanded_angles) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.5, 0.0};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);
    double psi_d{los_guidance_.calculate_psi_d(e.y_e)};
    double theta_d{los_guidance_.calculate_theta_d(e.z_e)};
    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(psi_d, 0.0);
    EXPECT_GT(psi_d, -1.57);
    // Pitch cmd should be zero
    EXPECT_NEAR(theta_d, 0.0, tol);
}

// Test commanded angles when drone is to the left of the track
TEST_F(LOSTests, T07_test_commanded_angles) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, -0.5, 0.0};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);
    double psi_d{los_guidance_.calculate_psi_d(e.y_e)};
    double theta_d{los_guidance_.calculate_theta_d(e.z_e)};
    // Heading cmd should be between 0 and pi/2
    EXPECT_GT(psi_d, 0.0);
    EXPECT_LT(psi_d, 1.57);
    // Pitch cmd should be zero
    EXPECT_NEAR(theta_d, 0.0, tol);
}

// Test commanded angles when drone is under the track
TEST_F(LOSTests, T08_test_commanded_angles) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.0, 0.5};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);
    double psi_d{los_guidance_.calculate_psi_d(e.y_e)};
    double theta_d{los_guidance_.calculate_theta_d(e.z_e)};
    // Heading cmd should be 0
    EXPECT_NEAR(psi_d, 0.0, tol);
    // Pitch cmd should be between 0 and pi/2
    EXPECT_GT(theta_d, 0.0);
    EXPECT_LT(theta_d, 1.57);
}

// Test commanded angles when drone is over the track
TEST_F(LOSTests, T09_test_commanded_angles) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.0, -0.5};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);
    double psi_d{los_guidance_.calculate_psi_d(e.y_e)};
    double theta_d{los_guidance_.calculate_theta_d(e.z_e)};
    // Heading cmd should be 0
    EXPECT_NEAR(psi_d, 0.0, tol);
    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(theta_d, 0.0);
    EXPECT_GT(theta_d, -1.57);
}

// Test commanded angles when drone is over and to the right of the track
TEST_F(LOSTests, T10_test_commanded_angles) {
    LOS::Point prev{0.0, 0.0, 0.0};
    LOS::Point next{1.0, 0.0, 0.0};
    los_guidance_.update_angles(prev, next);
    LOS::Point current{0.0, 0.5, -0.5};
    LOS::CrossTrackError e =
        los_guidance_.calculate_crosstrack_error(prev, current);
    double psi_d{los_guidance_.calculate_psi_d(e.y_e)};
    double theta_d{los_guidance_.calculate_theta_d(e.z_e)};
    // Heading cmd should be between -pi/2 and 0
    EXPECT_LT(psi_d, 0.0);
    EXPECT_GT(psi_d, -1.57);
    // Pitch cmd should be between -pi/2 and 0
    EXPECT_LT(theta_d, 0.0);
    EXPECT_GT(theta_d, -1.57);
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

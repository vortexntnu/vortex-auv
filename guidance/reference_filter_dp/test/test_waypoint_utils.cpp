#include <gtest/gtest.h>
#include <cmath>
#include "reference_filter_dp/lib/waypoint_utils.hpp"

namespace vortex::guidance {

// --- apply_mode_logic tests ---

TEST(ApplyModeLogic, FullPoseReturnsInputUnchanged) {
    Eigen::Vector6d r_in;
    r_in << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
    Eigen::Vector6d state = Eigen::Vector6d::Zero();

    Eigen::Vector6d result =
        apply_mode_logic(r_in, WaypointMode::FULL_POSE, state);

    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(result(i), r_in(i));
    }
}

TEST(ApplyModeLogic, OnlyPositionKeepsOrientationFromState) {
    Eigen::Vector6d r_in;
    r_in << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
    Eigen::Vector6d state;
    state << 10.0, 20.0, 30.0, 0.4, 0.5, 0.6;

    Eigen::Vector6d result =
        apply_mode_logic(r_in, WaypointMode::ONLY_POSITION, state);

    EXPECT_DOUBLE_EQ(result(0), 1.0);
    EXPECT_DOUBLE_EQ(result(1), 2.0);
    EXPECT_DOUBLE_EQ(result(2), 3.0);
    EXPECT_DOUBLE_EQ(result(3), 0.4);
    EXPECT_DOUBLE_EQ(result(4), 0.5);
    EXPECT_DOUBLE_EQ(result(5), 0.6);
}

TEST(ApplyModeLogic, ForwardHeadingComputesYawFromDelta) {
    Eigen::Vector6d r_in;
    r_in << 1.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Vector6d state = Eigen::Vector6d::Zero();

    Eigen::Vector6d result =
        apply_mode_logic(r_in, WaypointMode::FORWARD_HEADING, state);

    double expected_yaw = std::atan2(1.0, 1.0);  // pi/4
    EXPECT_DOUBLE_EQ(result(0), 1.0);
    EXPECT_DOUBLE_EQ(result(1), 1.0);
    EXPECT_DOUBLE_EQ(result(3), 0.0);
    EXPECT_DOUBLE_EQ(result(4), 0.0);
    EXPECT_NEAR(result(5), expected_yaw, 1e-12);
}

TEST(ApplyModeLogic, OnlyOrientationKeepsPositionFromState) {
    Eigen::Vector6d r_in;
    r_in << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
    Eigen::Vector6d state;
    state << 10.0, 20.0, 30.0, 0.4, 0.5, 0.6;

    Eigen::Vector6d result =
        apply_mode_logic(r_in, WaypointMode::ONLY_ORIENTATION, state);

    EXPECT_DOUBLE_EQ(result(0), 10.0);
    EXPECT_DOUBLE_EQ(result(1), 20.0);
    EXPECT_DOUBLE_EQ(result(2), 30.0);
    EXPECT_DOUBLE_EQ(result(3), 0.1);
    EXPECT_DOUBLE_EQ(result(4), 0.2);
    EXPECT_DOUBLE_EQ(result(5), 0.3);
}

// --- has_converged tests ---

TEST(HasConverged, FullPoseBelowThreshold) {
    Eigen::Vector6d measured;
    measured << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
    Eigen::Vector6d reference = measured;
    reference(0) += 0.001;

    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::FULL_POSE, 0.1));
}

TEST(HasConverged, FullPoseAboveThreshold) {
    Eigen::Vector6d measured = Eigen::Vector6d::Zero();
    Eigen::Vector6d reference;
    reference << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

    EXPECT_FALSE(
        has_converged(measured, reference, WaypointMode::FULL_POSE, 0.1));
}

TEST(HasConverged, OnlyPositionIgnoresOrientation) {
    Eigen::Vector6d measured;
    measured << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0;
    Eigen::Vector6d reference;
    reference << 1.0, 2.0, 3.0, 1.0, 1.0, 1.0;

    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::ONLY_POSITION, 0.1));
}

TEST(HasConverged, OnlyOrientationIgnoresPosition) {
    Eigen::Vector6d measured;
    measured << 100.0, 200.0, 300.0, 0.1, 0.2, 0.3;
    Eigen::Vector6d reference;
    reference << 0.0, 0.0, 0.0, 0.1, 0.2, 0.3;

    EXPECT_TRUE(has_converged(measured, reference,
                              WaypointMode::ONLY_ORIENTATION, 0.1));
}

TEST(HasConverged, ForwardHeadingUsesPositionAndYawOnly) {
    Eigen::Vector6d measured;
    measured << 1.0, 2.0, 3.0, 0.5, 0.5, 0.1;
    Eigen::Vector6d reference;
    reference << 1.0, 2.0, 3.0, 0.0, 0.0, 0.1;

    // Roll and pitch differ but should be ignored
    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::FORWARD_HEADING, 0.1));
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

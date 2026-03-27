#include <gtest/gtest.h>
#include <cmath>
#include "reference_filter_dp_quat/lib/waypoint_utils.hpp"

namespace vortex::guidance {

// --- compute_waypoint_goal tests ---

TEST(ComputeWaypointGoal, FullPoseReturnsInputUnchanged) {
    Pose incoming{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    Pose current{};

    Pose result =
        compute_waypoint_goal(incoming, WaypointMode::FULL_POSE, current);

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, 3.0);
    EXPECT_DOUBLE_EQ(result.qw, 1.0);
    EXPECT_DOUBLE_EQ(result.qx, 0.0);
    EXPECT_DOUBLE_EQ(result.qy, 0.0);
    EXPECT_DOUBLE_EQ(result.qz, 0.0);
}

TEST(ComputeWaypointGoal, OnlyPositionKeepsOrientationFromState) {
    Pose incoming{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};

    // Current state with a non-identity orientation (90 deg about Z)
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Pose current = Pose::from_eigen(Eigen::Vector3d(10.0, 20.0, 30.0), q);

    Pose result =
        compute_waypoint_goal(incoming, WaypointMode::ONLY_POSITION, current);

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, 3.0);

    // Orientation should match current state
    Eigen::Quaterniond result_q = result.ori_quaternion();
    EXPECT_TRUE(result_q.isApprox(q.normalized(), 1e-12));
}

TEST(ComputeWaypointGoal, ForwardHeadingComputesYawFromDelta) {
    Pose incoming{1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    Pose current{};

    Pose result =
        compute_waypoint_goal(incoming, WaypointMode::FORWARD_HEADING, current);

    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_DOUBLE_EQ(result.y, 1.0);

    // Expected yaw = atan2(1, 1) = pi/4
    double expected_yaw = std::atan2(1.0, 1.0);
    Eigen::Quaterniond expected_q(
        Eigen::AngleAxisd(expected_yaw, Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond result_q = result.ori_quaternion();
    EXPECT_TRUE(result_q.isApprox(expected_q.normalized(), 1e-12));
}

TEST(ComputeWaypointGoal, OnlyOrientationKeepsPositionFromState) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    Pose incoming = Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q);
    Pose current{10.0, 20.0, 30.0, 1.0, 0.0, 0.0, 0.0};

    Pose result = compute_waypoint_goal(
        incoming, WaypointMode::ONLY_ORIENTATION, current);

    EXPECT_DOUBLE_EQ(result.x, 10.0);
    EXPECT_DOUBLE_EQ(result.y, 20.0);
    EXPECT_DOUBLE_EQ(result.z, 30.0);

    Eigen::Quaterniond result_q = result.ori_quaternion();
    EXPECT_TRUE(result_q.isApprox(q.normalized(), 1e-12));
}

// --- has_converged tests ---

TEST(HasConverged, FullPoseBelowThreshold) {
    Pose measured{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    Pose reference{1.001, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};

    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::FULL_POSE, 0.1));
}

TEST(HasConverged, FullPoseAboveThreshold) {
    Pose measured{};
    Pose reference{1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0};

    EXPECT_FALSE(
        has_converged(measured, reference, WaypointMode::FULL_POSE, 0.1));
}

TEST(HasConverged, OnlyPositionIgnoresOrientation) {
    Pose measured{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};

    // Same position, very different orientation
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    Pose reference = Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q);

    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::ONLY_POSITION, 0.1));
}

TEST(HasConverged, OnlyOrientationIgnoresPosition) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
    Pose measured = Pose::from_eigen(Eigen::Vector3d(100.0, 200.0, 300.0), q);
    Pose reference = Pose::from_eigen(Eigen::Vector3d(0.0, 0.0, 0.0), q);

    EXPECT_TRUE(has_converged(measured, reference,
                              WaypointMode::ONLY_ORIENTATION, 0.1));
}

TEST(HasConverged, ForwardHeadingUsesPositionAndYawOnly) {
    // Same position and yaw, but different roll (single axis keeps error
    // purely in x-component, so z-component of quaternion_error is zero)
    Eigen::Quaterniond q_measured =
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_reference(
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));

    Pose measured =
        Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q_measured);
    Pose reference =
        Pose::from_eigen(Eigen::Vector3d(1.0, 2.0, 3.0), q_reference);

    // Roll differs but should be ignored in FORWARD_HEADING mode
    EXPECT_TRUE(
        has_converged(measured, reference, WaypointMode::FORWARD_HEADING, 0.1));
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

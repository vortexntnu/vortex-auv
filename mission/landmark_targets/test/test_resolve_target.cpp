#include <gtest/gtest.h>
#include <vortex/utils/waypoint_utils.hpp>
#include "landmark_targets/landmark_target.hpp"
#include "test_utils.hpp"

namespace vortex::mission {

using test::make_pose;
using test::yaw_of;

namespace {

MapLandmark landmark(const Pose& pose, bool has_orientation = true) {
    MapLandmark lm;
    lm.id = 1;
    lm.pose = pose;
    lm.has_orientation = has_orientation;
    return lm;
}

}  // namespace

TEST(ResolveTarget, OdomAxesMatchesApplyPoseOffset) {
    const Pose lm_pose = make_pose(4.0, -2.0, 1.5, 0.1, -0.2, 0.9);
    const Pose offset = make_pose(-2.0, 0.5, -1.0, 0.0, 0.0, 0.4);

    TargetSpec spec;
    spec.offset = offset;
    spec.frame = OffsetFrame::LANDMARK_ODOM_AXES;

    const Pose result = resolve_target(landmark(lm_pose, false), spec);
    const Pose expected =
        vortex::utils::waypoints::apply_pose_offset(lm_pose, offset);

    EXPECT_DOUBLE_EQ(result.x, expected.x);
    EXPECT_DOUBLE_EQ(result.y, expected.y);
    EXPECT_DOUBLE_EQ(result.z, expected.z);
    EXPECT_NEAR(result.qw, expected.qw, 1e-12);
    EXPECT_NEAR(result.qz, expected.qz, 1e-12);
}

TEST(ResolveTarget, LandmarkFrameOffsetIsInFrontOfTheObject) {
    // Landmark facing +y (yaw 90 deg): its +X (out of the front) points along
    // odom +y. "2 m in front, looking at the object" is at y = y_lm + 2.
    const Pose lm_pose = make_pose(10.0, 5.0, 2.0, 0.0, 0.0, M_PI_2);

    TargetSpec spec;
    spec.frame = OffsetFrame::LANDMARK;
    spec.offset = make_pose(2.0, 0.0, 0.0, 0.0, 0.0, M_PI);

    const Pose result = resolve_target(landmark(lm_pose), spec);

    EXPECT_NEAR(result.x, 10.0, 1e-12);
    EXPECT_NEAR(result.y, 7.0, 1e-12);
    EXPECT_NEAR(result.z, 2.0, 1e-12);
    // Yaw pi relative to the object: the vehicle looks at it (heading -y).
    EXPECT_NEAR(std::abs(yaw_of(result)), M_PI_2, 1e-12);
    EXPECT_NEAR(std::sin(yaw_of(result)), -1.0, 1e-12);

    // Negative x is behind the object.
    spec.offset = make_pose(-1.5, 0.0, 0.0);
    const Pose behind = resolve_target(landmark(lm_pose), spec);
    EXPECT_NEAR(behind.y, 5.0 - 1.5, 1e-12);
}

TEST(ResolveTarget, LandmarkFrameNeedsOrientation) {
    TargetSpec spec;
    spec.frame = OffsetFrame::LANDMARK;
    EXPECT_THROW(resolve_target(landmark(make_pose(0, 0, 0), false), spec),
                 std::invalid_argument);
}

TEST(ResolveTarget, ToolArmShiftsBaseLinkWithRotatedTarget) {
    // The tool sits 0.3 m ahead of and 0.2 m below base_link. The tool must
    // end up 1 m in front of the landmark; the vehicle faces the landmark.
    const Pose lm_pose = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 3);

    TargetSpec spec;
    spec.frame = OffsetFrame::LANDMARK;
    spec.offset = make_pose(1.0, 0.0, 0.0, 0.0, 0.0, M_PI);
    spec.tool_arm = Eigen::Vector3d(0.3, 0.0, 0.2);

    const Pose base = resolve_target(landmark(lm_pose), spec);

    // Reconstruct the tool position from the base pose.
    const Eigen::Vector3d tool =
        base.pos_vector() + base.ori_quaternion() * spec.tool_arm;
    const Eigen::Vector3d expected_tool =
        lm_pose.ori_quaternion() * Eigen::Vector3d(1.0, 0.0, 0.0);
    EXPECT_NEAR((tool - expected_tool).norm(), 0.0, 1e-12);

    // Orientation applies to base_link (yaw pi relative to the landmark).
    EXPECT_NEAR(std::abs(std::remainder(yaw_of(base) - (M_PI / 3 + M_PI),
                                        2 * M_PI)),
                0.0, 1e-12);
}

TEST(ResolveTarget, ToolArmWithRolledVehicleAndLandmark) {
    const Pose lm_pose = make_pose(3.0, 1.0, 2.0, 0.2, 0.1, -0.5);

    TargetSpec spec;
    spec.frame = OffsetFrame::LANDMARK_ODOM_AXES;
    spec.offset = make_pose(0.0, 0.0, -0.5, 0.3, -0.1, 1.2);
    spec.tool_arm = Eigen::Vector3d(0.25, -0.1, 0.4);

    const Pose base = resolve_target(landmark(lm_pose, false), spec);
    const Pose no_arm_target =
        vortex::utils::waypoints::apply_pose_offset(lm_pose, spec.offset);

    const Eigen::Vector3d tool =
        base.pos_vector() + base.ori_quaternion() * spec.tool_arm;
    EXPECT_NEAR((tool - no_arm_target.pos_vector()).norm(), 0.0, 1e-12);
}

}  // namespace vortex::mission

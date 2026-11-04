#include <gtest/gtest.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include "waypoint_manager/frame_resolver.hpp"

namespace vortex::mission {

namespace {

Pose make_pose(double x, double y, double z, double roll, double pitch, double yaw) {
    const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return Pose::from_eigen(Eigen::Vector3d(x, y, z), q);
}

double yaw_of(const Pose& p) {
    const Eigen::Quaterniond q = p.ori_quaternion();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace

TEST(FrameResolver, WorldIsUnchanged) {
    const Pose target = make_pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    const Pose start = make_pose(10.0, 20.0, 30.0, 0.0, 0.0, 1.0);

    const Pose out = resolve_pose(target, GoalFrame::WORLD, start);

    EXPECT_DOUBLE_EQ(out.x, target.x);
    EXPECT_DOUBLE_EQ(out.y, target.y);
    EXPECT_DOUBLE_EQ(out.z, target.z);
    EXPECT_DOUBLE_EQ(out.qw, target.qw);
    EXPECT_DOUBLE_EQ(out.qz, target.qz);
}

TEST(FrameResolver, BodyRelativeWithVehicleFacingEast) {
    // Yaw 90 deg in NED: the vehicle's forward axis points along +y (east).
    const Pose start = make_pose(1.0, 2.0, 3.0, 0.0, 0.0, M_PI_2);

    const Pose forward =
        resolve_pose(make_pose(2.0, 0.0, 0.0, 0, 0, 0), GoalFrame::BODY_RELATIVE, start);
    EXPECT_NEAR(forward.x, 1.0, 1e-12);
    EXPECT_NEAR(forward.y, 4.0, 1e-12);
    EXPECT_NEAR(forward.z, 3.0, 1e-12);

    // Starboard (+y in the body frame) is -x in odom when facing east.
    const Pose starboard =
        resolve_pose(make_pose(0.0, 1.0, 0.0, 0, 0, 0), GoalFrame::BODY_RELATIVE, start);
    EXPECT_NEAR(starboard.x, 0.0, 1e-12);
    EXPECT_NEAR(starboard.y, 2.0, 1e-12);

    // "30 degrees to the right" adds to the current heading.
    const Pose turned = resolve_pose(make_pose(0.0, 0.0, 0.0, 0, 0, M_PI / 6),
                                     GoalFrame::BODY_RELATIVE, start);
    EXPECT_NEAR(yaw_of(turned), M_PI_2 + M_PI / 6, 1e-12);
    EXPECT_NEAR(turned.x, 1.0, 1e-12);
    EXPECT_NEAR(turned.y, 2.0, 1e-12);
}

TEST(FrameResolver, BodyRelativeWithRolledAndPitchedVehicle) {
    const Pose start = make_pose(0.0, 0.0, 0.0, 0.3, -0.2, 0.7);
    const Pose offset = make_pose(1.0, -2.0, 0.5, 0, 0, 0);

    const Pose out = resolve_pose(offset, GoalFrame::BODY_RELATIVE, start);

    const Eigen::Vector3d expected =
        start.ori_quaternion() * Eigen::Vector3d(1.0, -2.0, 0.5);
    EXPECT_NEAR(out.x, expected.x(), 1e-12);
    EXPECT_NEAR(out.y, expected.y(), 1e-12);
    EXPECT_NEAR(out.z, expected.z(), 1e-12);
    // The offset length is preserved.
    EXPECT_NEAR((out.pos_vector() - start.pos_vector()).norm(),
                std::sqrt(1.0 + 4.0 + 0.25), 1e-12);
}

TEST(FrameResolver, WorldRelativeIgnoresVehicleHeadingForPosition) {
    const Pose start = make_pose(1.0, 2.0, 3.0, 0.0, 0.0, M_PI_2);

    const Pose up =
        resolve_pose(make_pose(0.0, 0.0, -1.0, 0, 0, 0), GoalFrame::WORLD_RELATIVE, start);
    EXPECT_NEAR(up.x, 1.0, 1e-12);
    EXPECT_NEAR(up.y, 2.0, 1e-12);
    EXPECT_NEAR(up.z, 2.0, 1e-12);

    const Pose north =
        resolve_pose(make_pose(5.0, 0.0, 0.0, 0, 0, 0), GoalFrame::WORLD_RELATIVE, start);
    EXPECT_NEAR(north.x, 6.0, 1e-12);
    EXPECT_NEAR(north.y, 2.0, 1e-12);
}

TEST(FrameResolver, WorldRelativeRotatesAboutOdomAxes) {
    const Pose start = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.4);
    const Pose out = resolve_pose(make_pose(0.0, 0.0, 0.0, 0, 0, 0.5),
                                  GoalFrame::WORLD_RELATIVE, start);
    EXPECT_NEAR(yaw_of(out), 0.9, 1e-12);
    EXPECT_NEAR(out.ori_quaternion().norm(), 1.0, 1e-12);
}

}  // namespace vortex::mission

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>
#include <cmath>
#include "landmark_targets/geometry.hpp"
#include "test_utils.hpp"

namespace vortex::mission {

using test::make_pose;
using vortex::utils::types::Pose;

TEST(Geometry, ForwardDistanceFollowsVehicleHeading) {
    const Pose facing_east = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2);
    EXPECT_NEAR(forward_distance(facing_east, {0.0, 3.0, 0.0}), 3.0, 1e-12);
    EXPECT_NEAR(forward_distance(facing_east, {0.0, -2.0, 0.0}), -2.0, 1e-12);
    EXPECT_NEAR(forward_distance(facing_east, {4.0, 0.0, 0.0}), 0.0, 1e-12);
}

TEST(Geometry, SideOfUsesHeading) {
    const Pose facing_north = make_pose(0.0, 0.0, 0.0);
    EXPECT_EQ(side_of(facing_north, {0.0, 1.0, 0.0}), Side::RIGHT);   // east
    EXPECT_EQ(side_of(facing_north, {0.0, -1.0, 0.0}), Side::LEFT);   // west

    const Pose facing_east = make_pose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2);
    EXPECT_EQ(side_of(facing_east, {1.0, 0.0, 0.0}), Side::LEFT);     // north
    EXPECT_EQ(side_of(facing_east, {-1.0, 0.0, 0.0}), Side::RIGHT);   // south
}

TEST(Geometry, PerpendicularHeadingPicksTheCloserOne) {
    // Line along +x: perpendiculars are +y (pi/2) and -y (-pi/2).
    const Eigen::Vector2d a(0.0, 0.0);
    const Eigen::Vector2d b(2.0, 0.0);
    EXPECT_NEAR(perpendicular_heading(a, b, 0.3), M_PI_2, 1e-12);
    EXPECT_NEAR(perpendicular_heading(a, b, -0.3), -M_PI_2, 1e-12);
    // Just past +-pi: -pi/2 is 90 deg away, +pi/2 too; either is fine, but the
    // result must be a valid perpendicular.
    const double h = perpendicular_heading(a, b, M_PI);
    EXPECT_NEAR(std::abs(std::sin(h)), 1.0, 1e-12);
    EXPECT_NEAR(std::cos(h), 0.0, 1e-12);
}

TEST(CourseFrame, RoundTripAndAxes) {
    CourseFrame course;
    course.origin = Eigen::Vector2d(10.0, 5.0);
    course.through_yaw = 0.0;  // through the gate = odom north (+x)
    course.state = CourseState::GATE_LOCKED;

    // 3 m through the gate, 2 m to the LEFT (west = -y in NED).
    const Eigen::Vector2d odom = from_course(course, {3.0, 2.0});
    EXPECT_NEAR(odom.x(), 13.0, 1e-12);
    EXPECT_NEAR(odom.y(), 3.0, 1e-12);
    const Eigen::Vector2d back = to_course(course, odom);
    EXPECT_NEAR(back.x(), 3.0, 1e-12);
    EXPECT_NEAR(back.y(), 2.0, 1e-12);
}

TEST(CourseFrame, GateRotated90Degrees) {
    CourseFrame course;
    course.origin = Eigen::Vector2d(1.0, 1.0);
    course.through_yaw = M_PI_2;  // through the gate = odom east (+y)
    course.state = CourseState::GATE_LOCKED;

    // 5 m through the gate is 5 m east; "left" of an east-facing course is
    // north (+x in odom).
    const Eigen::Vector2d through = from_course(course, {5.0, 0.0});
    EXPECT_NEAR(through.x(), 1.0, 1e-12);
    EXPECT_NEAR(through.y(), 6.0, 1e-12);
    const Eigen::Vector2d left = from_course(course, {0.0, 2.0});
    EXPECT_NEAR(left.x(), 3.0, 1e-12);
    EXPECT_NEAR(left.y(), 1.0, 1e-12);

    const Eigen::Vector2d c = to_course(course, {3.0, 6.0});
    EXPECT_NEAR(c.x(), 5.0, 1e-12);
    EXPECT_NEAR(c.y(), 2.0, 1e-12);

    // The side of a point relative to the course matches side_of for a
    // vehicle heading along the course.
    const Pose vehicle = make_pose(1.0, 1.0, 0.0, 0.0, 0.0, course.through_yaw);
    EXPECT_EQ(side_of(vehicle, {3.0, 1.0, 0.0}), Side::LEFT);
}

}  // namespace vortex::mission

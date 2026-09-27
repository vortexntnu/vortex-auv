#include <gtest/gtest.h>
#include <cmath>
#include "landmark_targets/slalom.hpp"
#include "test_utils.hpp"

namespace vortex::mission {

using vortex::utils::types::Pose;
using test::yaw_of;

TEST(AvoidSlalom, WaypointsInTheCourseFrame) {
    CourseFrame course;
    course.origin = Eigen::Vector2d(0.0, 0.0);
    course.through_yaw = 0.0;
    course.state = CourseState::GATE_LOCKED;

    // The last layer was at course (8, 1.0) (y to the right): more room
    // towards y = -6 (7 m) than towards y = +6 (5 m).
    const Eigen::Vector2d reference = from_course(course, {8.0, 1.0});
    const auto wps = avoid_slalom_waypoints(course, reference, -6.0, 6.0, 2.5, 0.7);
    ASSERT_EQ(wps.size(), 3u);

    const auto c0 = to_course(course, wps[0].pos_vector().head<2>());
    const auto c1 = to_course(course, wps[1].pos_vector().head<2>());
    const auto c2 = to_course(course, wps[2].pos_vector().head<2>());
    // y_side = (1 + -6) / 2 = -2.5.
    EXPECT_NEAR(c0.x(), 8.0, 1e-9);
    EXPECT_NEAR(c0.y(), -2.5, 1e-9);
    EXPECT_NEAR(c1.x(), 2.5, 1e-9);
    EXPECT_NEAR(c1.y(), -2.5, 1e-9);
    EXPECT_NEAR(c2.x(), 2.5, 1e-9);
    EXPECT_NEAR(c2.y(), 0.0, 1e-9);
    // Course y -2.5 with through_yaw 0 is 2.5 m west (odom -y).
    EXPECT_NEAR(wps[0].y, -2.5, 1e-9);
    // Looking back at the gate.
    EXPECT_NEAR(std::abs(yaw_of(wps[2])), M_PI, 1e-9);
    EXPECT_NEAR(wps[1].z, 0.7, 1e-12);
}

TEST(AvoidSlalom, CourseRotated90DegreesWithMoreRoomOnTheRight) {
    CourseFrame course;
    course.origin = Eigen::Vector2d(10.0, 5.0);
    course.through_yaw = M_PI_2;  // through the gate = odom +y (east)
    course.state = CourseState::GATE_LOCKED;

    const Eigen::Vector2d reference = from_course(course, {6.0, -1.0});
    const auto wps = avoid_slalom_waypoints(course, reference, -6.0, 6.0);
    ASSERT_EQ(wps.size(), 3u);
    // Room: 5 m towards -6, 7 m towards +6 -> y_side = (-1 + 6) / 2 = 2.5.
    const auto c0 = to_course(course, wps[0].pos_vector().head<2>());
    EXPECT_NEAR(c0.x(), 6.0, 1e-9);
    EXPECT_NEAR(c0.y(), 2.5, 1e-9);
    // In odom: 6 m east of the gate and 2.5 m to the right of an east-facing
    // course, i.e. south (-x).
    EXPECT_NEAR(wps[0].x, 10.0 - 2.5, 1e-9);
    EXPECT_NEAR(wps[0].y, 5.0 + 6.0, 1e-9);
    // Looking back at the gate: yaw = through_yaw + pi = -pi/2.
    EXPECT_NEAR(std::remainder(yaw_of(wps[2]) - (-M_PI_2), 2 * M_PI), 0.0, 1e-9);
}

}  // namespace vortex::mission

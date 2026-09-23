#include <gtest/gtest.h>
#include <cmath>
#include "landmark_targets/slalom.hpp"
#include "test_utils.hpp"

namespace vortex::mission {

using test::make_pose;
using vortex::utils::types::Pose;
using test::yaw_of;

namespace {

Pipe pipe(int id, double x, double y) { return {id, Eigen::Vector3d(x, y, 2.0)}; }

// One layer of the course, along +x: white at y = -2, red at y = 0, white at
// y = 2 (x is the down-course position of the layer).
struct Layer {
    Pipe white_left, red, white_right;
};

Layer layer(int first_id, double x, double y_center = 0.0) {
    return {pipe(first_id, x, y_center - 2.0), pipe(first_id + 1, x, y_center),
            pipe(first_id + 2, x, y_center + 2.0)};
}

}  // namespace

TEST(MatchPipes, GapBetweenRedAndTheWhiteOnTheGateSide) {
    const Layer l = layer(1, 5.0);
    // Vehicle at the origin facing +x: left is -y (NED, z down).
    const Pose vehicle = make_pose(0.0, 0.0, 2.0);

    const auto gap_left = match_pipes({l.red}, {l.white_left, l.white_right}, vehicle,
                                      Side::LEFT, {});
    ASSERT_TRUE(gap_left.has_value());
    EXPECT_EQ(gap_left->red_id, 2);
    EXPECT_NEAR(gap_left->position.x(), 5.0, 1e-9);
    EXPECT_NEAR(gap_left->position.y(), -1.0, 1e-9);  // between red and left white
    // Through the gap along the course (+x), turned 15 deg towards the red pipe.
    EXPECT_NEAR(std::abs(gap_left->heading), 15.0 * M_PI / 180.0, 1e-9);
    EXPECT_GT(gap_left->heading, 0.0);  // gap is left of red: turn right (+yaw)
    EXPECT_NEAR(gap_left->offset_from_red.y(), -1.0, 1e-9);

    const auto gap_right = match_pipes({l.red}, {l.white_left, l.white_right}, vehicle,
                                       Side::RIGHT, {});
    ASSERT_TRUE(gap_right.has_value());
    EXPECT_NEAR(gap_right->position.y(), 1.0, 1e-9);
    EXPECT_LT(gap_right->heading, 0.0);
}

TEST(MatchPipes, CourseRotated90Degrees) {
    // The same layer along odom +y instead: the vehicle faces +y.
    Layer l;
    l.white_left = pipe(1, 2.0, 5.0);   // left of a +y-facing vehicle is +x
    l.red = pipe(2, 0.0, 5.0);
    l.white_right = pipe(3, -2.0, 5.0);
    const Pose vehicle = make_pose(0.0, 0.0, 2.0, 0.0, 0.0, M_PI_2);

    const auto gap = match_pipes({l.red}, {l.white_left, l.white_right}, vehicle,
                                 Side::LEFT, {});
    ASSERT_TRUE(gap.has_value());
    EXPECT_NEAR(gap->position.x(), 1.0, 1e-9);
    EXPECT_NEAR(gap->position.y(), 5.0, 1e-9);
    // Heading along +y, turned 15 deg.
    EXPECT_NEAR(std::abs(gap->heading - M_PI_2), 15.0 * M_PI / 180.0, 1e-9);
}

TEST(MatchPipes, PassedRedPipesAndPipesBehindAreIgnored) {
    const Layer near_layer = layer(1, 5.0);
    const Layer far_layer = layer(10, 9.0);
    const Pose vehicle = make_pose(0.0, 0.0, 2.0);
    std::vector<Pipe> whites = {near_layer.white_left, near_layer.white_right,
                                far_layer.white_left, far_layer.white_right};

    // Nearest red first.
    auto gap = match_pipes({near_layer.red, far_layer.red}, whites, vehicle, Side::LEFT, {});
    ASSERT_TRUE(gap.has_value());
    EXPECT_EQ(gap->red_id, 2);

    // Excluded by id once passed.
    gap = match_pipes({near_layer.red, far_layer.red}, whites, vehicle, Side::LEFT, {2});
    ASSERT_TRUE(gap.has_value());
    EXPECT_EQ(gap->red_id, 11);

    // A red pipe less than 1 m in front is not a candidate.
    gap = match_pipes({pipe(20, 0.5, 0.0)}, whites, vehicle, Side::LEFT, {});
    EXPECT_FALSE(gap.has_value());
    // No red pipe at all.
    EXPECT_FALSE(match_pipes({}, whites, vehicle, Side::LEFT, {}).has_value());
}

TEST(MatchPipes, ShortcutUsesThePreviousOffset) {
    const Layer second = layer(10, 9.0, 0.5);
    const Pose vehicle = make_pose(6.0, 0.0, 2.0);
    SlalomOffset offset;
    offset.offset = Eigen::Vector2d(0.0, -1.0);
    offset.heading = 0.2;

    // No white pipes needed.
    const auto gap = match_pipes({second.red}, {}, vehicle, Side::LEFT, {}, offset);
    ASSERT_TRUE(gap.has_value());
    EXPECT_NEAR(gap->position.x(), 9.0, 1e-9);
    EXPECT_NEAR(gap->position.y(), 0.5 - 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(gap->heading, 0.2);
}

TEST(MatchPipes, WhitePipesCloseToTheRedOneAreIgnored) {
    const Pose vehicle = make_pose(0.0, 0.0, 2.0);
    // Only a white pipe 0.3 m from the red one: not a usable neighbour.
    const auto gap = match_pipes({pipe(2, 5.0, 0.0)}, {pipe(1, 5.0, -0.3)}, vehicle,
                                 Side::LEFT, {});
    EXPECT_FALSE(gap.has_value());
}

TEST(MatchPipes, OneWhitePipeOnTheRightSideGivesTheMidpoint) {
    const Pose vehicle = make_pose(0.0, 0.0, 2.0);
    // Gap on the left; the visible white pipe is on the left: midpoint.
    const auto gap = match_pipes({pipe(2, 5.0, 0.0)}, {pipe(1, 5.0, -2.0)}, vehicle,
                                 Side::LEFT, {});
    ASSERT_TRUE(gap.has_value());
    EXPECT_NEAR(gap->position.x(), 5.0, 1e-9);
    EXPECT_NEAR(gap->position.y(), -1.0, 1e-9);
}

TEST(MatchPipes, OneWhitePipeOnTheWrongSideIsMirrored) {
    const Pose vehicle = make_pose(0.0, 0.0, 2.0);
    // Gap on the left, but the only white pipe is on the right: the gap is on
    // the other side of the red pipe, half a pipe distance away.
    const auto gap = match_pipes({pipe(2, 5.0, 0.0)}, {pipe(3, 5.0, 2.0)}, vehicle,
                                 Side::LEFT, {});
    ASSERT_TRUE(gap.has_value());
    EXPECT_NEAR(gap->position.x(), 5.0, 1e-9);
    EXPECT_NEAR(gap->position.y(), -1.0, 1e-9);  // red - (white - red) / 2
}

TEST(MatchPipes, RedPipeMustLieOnTheLineBetweenTheWhitePairs) {
    const Pose vehicle = make_pose(0.0, 0.0, 2.0);
    // The whites are a layer away from the red pipe (1.5 m off the line):
    // not a pair for this red pipe.
    const auto gap = match_pipes({pipe(2, 5.0, 0.0)},
                                 {pipe(1, 6.5, -2.0), pipe(3, 6.5, 2.0)}, vehicle,
                                 Side::LEFT, {});
    // Falls back to one white pipe (the nearest), not the pair.
    ASSERT_TRUE(gap.has_value());
    EXPECT_NEAR(gap->position.x(), 5.0 + 0.0, 1.0);
}

TEST(AvoidSlalom, WaypointsInTheCourseFrame) {
    CourseFrame course;
    course.origin = Eigen::Vector2d(0.0, 0.0);
    course.through_yaw = 0.0;
    course.state = CourseState::GATE_LOCKED;

    // The last layer was at course (8, 1.0): more room to the right (y -6).
    // NED: course y is to the left; odom y = -course y when through_yaw = 0.
    const Eigen::Vector2d reference = from_course(course, {8.0, 1.0});
    const auto wps = avoid_slalom_waypoints(course, reference, 6.0, -6.0, 2.5, 0.7);
    ASSERT_EQ(wps.size(), 3u);

    const auto c0 = to_course(course, wps[0].pos_vector().head<2>());
    const auto c1 = to_course(course, wps[1].pos_vector().head<2>());
    const auto c2 = to_course(course, wps[2].pos_vector().head<2>());
    // Room: left 5, right 7 -> y_side = (1 + -6) / 2 = -2.5.
    EXPECT_NEAR(c0.x(), 8.0, 1e-9);
    EXPECT_NEAR(c0.y(), -2.5, 1e-9);
    EXPECT_NEAR(c1.x(), 2.5, 1e-9);
    EXPECT_NEAR(c1.y(), -2.5, 1e-9);
    EXPECT_NEAR(c2.x(), 2.5, 1e-9);
    EXPECT_NEAR(c2.y(), 0.0, 1e-9);
    // Looking back at the gate.
    EXPECT_NEAR(std::abs(yaw_of(wps[2])), M_PI, 1e-9);
    EXPECT_NEAR(wps[1].z, 0.7, 1e-12);
}

TEST(AvoidSlalom, CourseRotated90DegreesWithMoreRoomOnTheLeft) {
    CourseFrame course;
    course.origin = Eigen::Vector2d(10.0, 5.0);
    course.through_yaw = M_PI_2;  // through the gate = odom +y
    course.state = CourseState::GATE_LOCKED;

    const Eigen::Vector2d reference = from_course(course, {6.0, -1.0});
    const auto wps = avoid_slalom_waypoints(course, reference, 6.0, -6.0);
    ASSERT_EQ(wps.size(), 3u);
    // Room: left 7, right 5 -> y_side = (-1 + 6) / 2 = 2.5 (to the left).
    const auto c0 = to_course(course, wps[0].pos_vector().head<2>());
    EXPECT_NEAR(c0.x(), 6.0, 1e-9);
    EXPECT_NEAR(c0.y(), 2.5, 1e-9);
    // In odom: 6 m along +y from the gate, 2.5 m to the left (= +x, north)... of an east-facing course.
    EXPECT_NEAR(wps[0].x, 10.0 + 2.5, 1e-9);
    EXPECT_NEAR(wps[0].y, 5.0 + 6.0, 1e-9);
    // Looking back at the gate: yaw = through_yaw + pi = -pi/2.
    EXPECT_NEAR(std::remainder(yaw_of(wps[2]) - (-M_PI_2), 2 * M_PI), 0.0, 1e-9);
}

}  // namespace vortex::mission

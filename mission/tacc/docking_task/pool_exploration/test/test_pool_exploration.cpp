#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>
#include <cmath>

#include <pool_exploration/pool_exploration.hpp>
#include <vortex/utils/types.hpp>

namespace vortex::pool_exploration::test {

using vortex::utils::types::LineSegment2D;
using vortex::utils::types::Point2D;

namespace {

Point2D make_point(float x, float y) {
    return Point2D{.x = x, .y = y};
}

LineSegment2D make_line(float x0, float y0, float x1, float y1) {
    return LineSegment2D{
        .p0 = make_point(x0, y0),
        .p1 = make_point(x1, y1),
    };
}

PoolExplorationPlannerConfig make_config() {
    PoolExplorationPlannerConfig config{};

    // Avstander for gyldige vegger
    config.min_wall_distance_m = 0.2f;
    config.max_wall_distance_m = 10.0f;

    // Klassifisering
    // Right wall: y må være negativ nok i NED
    config.right_wall_max_y_m = -0.1f;

    // Far/front wall: x må være positiv nok
    config.far_wall_min_x_m = 0.1f;

    // Vinkler
    config.parallel_heading_angle_threshold_rad = 20.0f * static_cast<float>(M_PI) / 180.0f;
    config.perpendicular_heading_angle_threshold_rad = 70.0f * static_cast<float>(M_PI) / 180.0f;

    // Hjørnevinkel
    config.min_corner_angle_rad = 70.0f * static_cast<float>(M_PI) / 180.0f;
    config.max_corner_angle_rad = 110.0f * static_cast<float>(M_PI) / 180.0f;

    // Docking-offset
    config.right_wall_offset_m = 1.0f;
    config.far_wall_offset_m = 1.5f;

    return config;
}

}  // namespace

class PoolExplorationPlannerTest : public ::testing::Test {
protected:
    PoolExplorationPlannerTest()
        : config(make_config()),
          planner(config) {}

    PoolExplorationPlannerConfig config;
    PoolExplorationPlanner planner;
};

TEST_F(PoolExplorationPlannerTest, FindCornerEstimates_FindsCornerFromPerpendicularWalls) {
    // Drone i origo, heading langs +x
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    // "Right wall" i NED: horisontal linje under drone (negativ y)
    // "Front wall": vertikal linje foran drone (positiv x)
    const LineSegment2D right_wall = make_line(0.0f, -2.0f, 10.0f, -2.0f);
    const LineSegment2D front_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f);

    const std::vector<LineSegment2D> lines{right_wall, front_wall};

    const auto corners = planner.find_corner_estimates(lines, drone_pos, drone_heading);

    ASSERT_EQ(corners.size(), 1u);

    EXPECT_NEAR(corners[0].corner_point.x(), 5.0f, 1e-4f);
    EXPECT_NEAR(corners[0].corner_point.y(), -2.0f, 1e-4f);
}

TEST_F(PoolExplorationPlannerTest, FindCornerEstimates_RejectsParallelWalls) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    // Begge linjer omtrent parallelle med heading -> skal ikke gi hjørne
    const LineSegment2D line0 = make_line(0.0f, -2.0f, 10.0f, -2.0f);
    const LineSegment2D line1 = make_line(0.0f, -4.0f, 10.0f, -4.0f);

    const std::vector<LineSegment2D> lines{line0, line1};

    const auto corners = planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST_F(PoolExplorationPlannerTest, FindCornerEstimates_RejectsWallsOutsideDistanceThreshold) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    // Right wall er for langt unna
    const LineSegment2D right_wall_too_far = make_line(0.0f, -20.0f, 10.0f, -20.0f);
    const LineSegment2D front_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f);

    const std::vector<LineSegment2D> lines{right_wall_too_far, front_wall};

    const auto corners = planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST_F(PoolExplorationPlannerTest, SelectBestCorner_ReturnsClosestCorner) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};

    CornerEstimate c0{
        .right_wall = make_line(0.0f, -2.0f, 10.0f, -2.0f),
        .far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f),
        .corner_point = Eigen::Vector2f{5.0f, -2.0f}
    };

    CornerEstimate c1{
        .right_wall = make_line(0.0f, -4.0f, 10.0f, -4.0f),
        .far_wall = make_line(8.0f, -5.0f, 8.0f, 5.0f),
        .corner_point = Eigen::Vector2f{8.0f, -4.0f}
    };

    const auto best = planner.select_best_corner({c1, c0}, drone_pos);

    EXPECT_NEAR(best.corner_point.x(), 5.0f, 1e-4f);
    EXPECT_NEAR(best.corner_point.y(), -2.0f, 1e-4f);
}

TEST_F(PoolExplorationPlannerTest, SelectBestCorner_ThrowsOnEmptyInput) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};

    EXPECT_THROW(
        planner.select_best_corner({}, drone_pos),
        std::runtime_error
    );
}

TEST_F(PoolExplorationPlannerTest, EstimateDockingPosition_OffsetsFromCornerTowardDroneSideOfWalls) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};

    // Hjørne i (5, -2)
    // Right wall: y = -2
    // Front wall: x = 5
    //
    // Med drone i (0,0):
    // - normal fra right wall mot drone peker opp: (0, +1)
    // - normal fra front wall mot drone peker venstre: (-1, 0)
    //
    // Forventet docking:
    // (5, -2) + 1.0*(0,1) + 1.5*(-1,0) = (3.5, -1.0)

    CornerEstimate corner{
        .right_wall = make_line(0.0f, -2.0f, 10.0f, -2.0f),
        .far_wall = make_line(5.0f, -5.0f, 5.0f, 5.0f),
        .corner_point = Eigen::Vector2f{5.0f, -2.0f}
    };

    const Eigen::Vector2f docking = planner.estimate_docking_position(corner, drone_pos);

    EXPECT_NEAR(docking.x(), 3.5f, 1e-4f);
    EXPECT_NEAR(docking.y(), -1.0f, 1e-4f);
}

TEST_F(PoolExplorationPlannerTest, FullPipeline_FindsCornerSelectsBestAndEstimatesDocking) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    // Ett riktig hjørne nærme
    const LineSegment2D right_wall_near = make_line(0.0f, -2.0f, 10.0f, -2.0f);
    const LineSegment2D front_wall_near = make_line(5.0f, -5.0f, 5.0f, 5.0f);

    // Ett riktig hjørne lenger unna
    const LineSegment2D right_wall_far = make_line(0.0f, -4.0f, 10.0f, -4.0f);
    const LineSegment2D front_wall_far = make_line(9.0f, -5.0f, 9.0f, 5.0f);

    // Litt støy som ikke skal bli brukt
    const LineSegment2D noise = make_line(-5.0f, 3.0f, -2.0f, 3.0f);

    const std::vector<LineSegment2D> lines{
        right_wall_near, front_wall_near,
        right_wall_far, front_wall_far,
        noise
    };

    const auto corners = planner.find_corner_estimates(lines, drone_pos, drone_heading);

    ASSERT_GE(corners.size(), 2u);

    const auto best = planner.select_best_corner(corners, drone_pos);
    const auto docking = planner.estimate_docking_position(best, drone_pos);

    EXPECT_NEAR(best.corner_point.x(), 5.0f, 1e-4f);
    EXPECT_NEAR(best.corner_point.y(), -2.0f, 1e-4f);

    EXPECT_NEAR(docking.x(), 3.5f, 1e-4f);
    EXPECT_NEAR(docking.y(), -1.0f, 1e-4f);
}

TEST_F(PoolExplorationPlannerTest, FindCornerEstimates_AcceptsApproximatelyPerpendicularWalls) {
    const Eigen::Vector2f drone_pos{0.0f, 0.0f};
    const float drone_heading = 0.0f;

    // Litt skjeve vegger, men fortsatt innenfor tersklene
    const LineSegment2D right_wall = make_line(0.0f, -2.0f, 10.0f, -1.5f);
    const LineSegment2D front_wall = make_line(5.0f, -5.0f, 5.5f, 5.0f);

    const std::vector<LineSegment2D> lines{right_wall, front_wall};

    const auto corners = planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_EQ(corners.size(), 1u);
}

}  // namespace vortex::pool_exploration::test
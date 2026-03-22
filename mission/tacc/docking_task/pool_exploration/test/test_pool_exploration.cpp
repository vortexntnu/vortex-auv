#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <vector>

#include <pool_exploration/pool_exploration.hpp>
#include <vortex/utils/types.hpp>

using namespace vortex::pool_exploration;

namespace
{

vortex::utils::types::LineSegment2D makeLine(float x0, float y0, float x1, float y1)
{
    vortex::utils::types::LineSegment2D line;
    line.p0.x = x0;
    line.p0.y = y0;
    line.p1.x = x1;
    line.p1.y = y1;
    return line;
}

PoolExplorationPlannerConfig makeDefaultConfig()
{
    PoolExplorationPlannerConfig config{};
    config.min_wall_distance_m = 0.5f;
    config.max_wall_distance_m = 20.0f;
    config.far_wall_heading_angle_threshold = 0.6f;  // ca 34 deg
    config.min_corner_angle_rad = 1.0f;
    config.max_corner_angle_rad = 1.6f;              // litt over pi/2
    config.right_dist = 0.1f;
    config.right_wall_offset_m = 1.0f;
    config.far_wall_offset_m = 1.0f;
    return config;
}

}  // namespace

// ------------------------------------------------------------
// Grunnleggende geometriske tester
// ------------------------------------------------------------

TEST(PoolExplorationPlannerNEDTest, ProjectDroneToHorizontalLine)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(2.0f, 3.0f);
    auto line = makeLine(0.0f, 0.0f, 10.0f, 0.0f);

    Eigen::Vector2f projection = planner.project_point_onto_line(drone_pos, line);

    EXPECT_NEAR(projection.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(projection.y(), 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, ProjectDroneToVerticalLine)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(4.0f, 2.0f);
    auto line = makeLine(1.0f, -5.0f, 1.0f, 5.0f);

    Eigen::Vector2f projection = planner.project_point_onto_line(drone_pos, line);

    EXPECT_NEAR(projection.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(projection.y(), 2.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, ProjectDroneToDegenerateLineReturnsPoint)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(4.0f, 2.0f);
    auto line = makeLine(1.0f, 1.0f, 1.0f, 1.0f);

    Eigen::Vector2f projection = planner.project_point_onto_line(drone_pos, line);

    EXPECT_NEAR(projection.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(projection.y(), 1.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, LineHeadingAngleDifferenceParallel)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line = makeLine(0.0f, 0.0f, 5.0f, 0.0f);
    float angle = planner.angle_between_line_and_heading(line, 0.0f);

    EXPECT_NEAR(angle, 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, LineHeadingAngleDifferenceParallelReversedOrder)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line = makeLine(5.0f, 0.0f, 0.0f, 0.0f);
    float angle = planner.angle_between_line_and_heading(line, 0.0f);

    EXPECT_NEAR(angle, 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, LineHeadingAngleDifferencePerpendicular)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line = makeLine(0.0f, 0.0f, 0.0f, 5.0f);
    float angle = planner.angle_between_line_and_heading(line, 0.0f);

    EXPECT_NEAR(angle, static_cast<float>(M_PI_2), 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, LineIntersectionFindsCorrectPoint)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line0 = makeLine(0.0f, 0.0f, 10.0f, 0.0f);
    auto line1 = makeLine(2.0f, -5.0f, 2.0f, 5.0f);

    Eigen::Vector2f intersection;
    bool intersects = planner.compute_line_intersection(line0, line1, intersection);

    EXPECT_TRUE(intersects);
    EXPECT_NEAR(intersection.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(intersection.y(), 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, LineIntersectionParallelReturnsFalse)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line0 = makeLine(0.0f, 0.0f, 10.0f, 0.0f);
    auto line1 = makeLine(0.0f, 1.0f, 10.0f, 1.0f);

    Eigen::Vector2f intersection;
    bool intersects = planner.compute_line_intersection(line0, line1, intersection);

    EXPECT_FALSE(intersects);
}

TEST(PoolExplorationPlannerNEDTest, AngleBetweenLinesPerpendicular)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line0 = makeLine(0.0f, 0.0f, 10.0f, 0.0f);
    auto line1 = makeLine(0.0f, 0.0f, 0.0f, 10.0f);

    float angle = planner.angle_between_lines(line0, line1);

    EXPECT_NEAR(angle, static_cast<float>(M_PI_2), 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, AngleBetweenLinesParallelReversedOrderStillZero)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line0 = makeLine(0.0f, 0.0f, 10.0f, 0.0f);
    auto line1 = makeLine(10.0f, 0.0f, 0.0f, 0.0f);

    float angle = planner.angle_between_lines(line0, line1);

    EXPECT_NEAR(angle, 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, SelectBestCornerChoosesClosestCorner)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    CornerEstimate c0;
    c0.corner_point = Eigen::Vector2f(10.0f, 10.0f);

    CornerEstimate c1;
    c1.corner_point = Eigen::Vector2f(2.0f, 1.0f);

    CornerEstimate c2;
    c2.corner_point = Eigen::Vector2f(5.0f, 5.0f);

    std::vector<CornerEstimate> corners{c0, c1, c2};
    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    CornerEstimate best = planner.select_best_corner(corners, drone_pos);

    EXPECT_NEAR(best.corner_point.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(best.corner_point.y(), 1.0f, 1e-5f);
}

// ------------------------------------------------------------
// NED-spesifikke tester for wall classification via find_corner_estimates()
// ------------------------------------------------------------
//
// Antagelse:
// x = North
// y = East
// heading = 0 -> peker nord
// heading = pi/2 -> peker øst
//
// right = (sin(h), -cos(h))
//
// Ved heading = 0:
//   forward = (1, 0)
//   right   = (0,-1)
//   => negativ y er høyre side
//
// Ved heading = pi/2:
//   forward = (0, 1)
//   right   = (1, 0)
//   => positiv x er høyre side
// ------------------------------------------------------------

TEST(PoolExplorationPlannerNEDTest, FindValidCornerHeadingZero_NegativeYIsRightWall)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = 0.0f;

    // Høyrevegg i NED ved heading=0: y < 0
    auto right_wall = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);
    auto far_wall   = makeLine(5.0f, -10.0f, 5.0f, 10.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{right_wall, far_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    ASSERT_EQ(corners.size(), 1u);
    EXPECT_NEAR(corners[0].corner_point.x(), 5.0f, 1e-4f);
    EXPECT_NEAR(corners[0].corner_point.y(), -2.0f, 1e-4f);
}

TEST(PoolExplorationPlannerNEDTest, FindValidCornerHeadingZero_PositiveYIsLeftWallNotRightWall)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = 0.0f;

    // Venstre vegg i NED ved heading=0: y > 0
    auto left_wall = makeLine(-10.0f, 2.0f, 10.0f, 2.0f);
    auto far_wall  = makeLine(5.0f, -10.0f, 5.0f, 10.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{left_wall, far_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST(PoolExplorationPlannerNEDTest, FindValidCornerHeadingZero_FarWallMustBeInFront)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = 0.0f;

    auto right_wall = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);

    // Denne er bak dronen siden x < 0 når heading=0
    auto behind_wall = makeLine(-5.0f, -10.0f, -5.0f, 10.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{right_wall, behind_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST(PoolExplorationPlannerNEDTest, FindValidCornerHeading90Deg_PositiveXIsRightWall)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = static_cast<float>(M_PI_2);

    // Ved heading = pi/2:
    // forward = (0,1)
    // right   = (1,0)
    //
    // Positiv x er høyre side
    auto right_wall = makeLine(2.0f, -10.0f, 2.0f, 10.0f);
    auto far_wall   = makeLine(-10.0f, 5.0f, 10.0f, 5.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{right_wall, far_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    ASSERT_EQ(corners.size(), 1u);
    EXPECT_NEAR(corners[0].corner_point.x(), 2.0f, 1e-4f);
    EXPECT_NEAR(corners[0].corner_point.y(), 5.0f, 1e-4f);
}

TEST(PoolExplorationPlannerNEDTest, FindValidCornerHeading90Deg_NegativeXIsLeftWallNotRightWall)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = static_cast<float>(M_PI_2);

    auto left_wall = makeLine(-2.0f, -10.0f, -2.0f, 10.0f);
    auto far_wall  = makeLine(-10.0f, 5.0f, 10.0f, 5.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{left_wall, far_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST(PoolExplorationPlannerNEDTest, FindValidCornerHeading90Deg_FarWallMustBeAheadInPositiveY)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = static_cast<float>(M_PI_2);

    auto right_wall = makeLine(2.0f, -10.0f, 2.0f, 10.0f);

    // Bak dronen når heading=pi/2, siden y < 0
    auto behind_wall = makeLine(-10.0f, -5.0f, 10.0f, -5.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{right_wall, behind_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST(PoolExplorationPlannerNEDTest, FindCornerRejectsWallOutsideDistanceLimits)
{
    PoolExplorationPlannerConfig config = makeDefaultConfig();
    config.max_wall_distance_m = 1.0f;

    PoolExplorationPlanner planner(config);

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = 0.0f;

    // y=-2 ligger for langt unna gitt max_wall_distance_m=1.0
    auto right_wall = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);
    auto far_wall   = makeLine(0.8f, -10.0f, 0.8f, 10.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{right_wall, far_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}

TEST(PoolExplorationPlannerNEDTest, FindCornerRejectsNonPerpendicularCornerAngle)
{
    PoolExplorationPlannerConfig config = makeDefaultConfig();
    config.min_corner_angle_rad = 1.4f;
    config.max_corner_angle_rad = 1.6f;

    PoolExplorationPlanner planner(config);

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = 0.0f;

    auto right_wall = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);

    // Ca 63.4 grader mot horisontal linje -> skal avvises
    auto diagonal_far_wall = makeLine(5.0f, -10.0f, 15.0f, 10.0f);

    std::vector<vortex::utils::types::LineSegment2D> lines{right_wall, diagonal_far_wall};

    std::vector<CornerEstimate> corners =
        planner.find_corner_estimates(lines, drone_pos, drone_heading);

    EXPECT_TRUE(corners.empty());
}
// ------------------------------------------------------------
// Tester for compute_normal_towards_point() og docking-estimat
// ------------------------------------------------------------

TEST(PoolExplorationPlannerNEDTest, ComputeNormalTowardsPointForHorizontalWall)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);
    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    Eigen::Vector2f normal = planner.compute_normal_towards_point(line, drone_pos);

    EXPECT_NEAR(normal.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(normal.y(), 1.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, ComputeNormalTowardsPointForVerticalWall)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line = makeLine(5.0f, -10.0f, 5.0f, 10.0f);
    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    Eigen::Vector2f normal = planner.compute_normal_towards_point(line, drone_pos);

    EXPECT_NEAR(normal.x(), -1.0f, 1e-5f);
    EXPECT_NEAR(normal.y(), 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, ComputeNormalTowardsPointIsIndependentOfLineEndpointOrdering)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line_a = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);
    auto line_b = makeLine(10.0f, -2.0f, -10.0f, -2.0f);
    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    Eigen::Vector2f normal_a = planner.compute_normal_towards_point(line_a, drone_pos);
    Eigen::Vector2f normal_b = planner.compute_normal_towards_point(line_b, drone_pos);

    EXPECT_NEAR(normal_a.x(), normal_b.x(), 1e-5f);
    EXPECT_NEAR(normal_a.y(), normal_b.y(), 1e-5f);
}

TEST(PoolExplorationPlannerNEDTest, EstimateDockingPositionFromNEDCorner)
{
    PoolExplorationPlannerConfig config = makeDefaultConfig();
    config.right_wall_offset_m = 1.0f;
    config.far_wall_offset_m = 2.0f;

    PoolExplorationPlanner planner(config);

    CornerEstimate corner;
    corner.right_wall   = makeLine(-10.0f, -2.0f, 10.0f, -2.0f);
    corner.far_wall     = makeLine(5.0f, -10.0f, 5.0f, 10.0f);
    corner.corner_point = Eigen::Vector2f(5.0f, -2.0f);

    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    Eigen::Vector2f docking = planner.estimate_docking_position(corner, drone_pos);

    // right_wall y=-2  -> normal mot drone = (0, +1)
    // far_wall   x= 5  -> normal mot drone = (-1, 0)
    //
    // docking = (5,-2) + 1*(0,1) + 2*(-1,0) = (3,-1)
    EXPECT_NEAR(docking.x(), 3.0f, 1e-4f);
    EXPECT_NEAR(docking.y(), -1.0f, 1e-4f);
}

// ------------------------------------------------------------
// Robusthetstester
// ------------------------------------------------------------

TEST(PoolExplorationPlannerNEDTest, SelectBestCornerThrowsWhenEmpty)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::vector<CornerEstimate> corners;
    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    EXPECT_THROW(planner.select_best_corner(corners, drone_pos), std::runtime_error);
}

TEST(PoolExplorationPlannerNEDTest, AngleBetweenDegenerateLineAndHeadingReturnsInfinity)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line = makeLine(1.0f, 1.0f, 1.0f, 1.0f);
    float angle = planner.angle_between_line_and_heading(line, 0.0f);

    EXPECT_TRUE(std::isinf(angle));
}

TEST(PoolExplorationPlannerNEDTest, AngleBetweenDegenerateLinesReturnsInfinity)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    auto line0 = makeLine(0.0f, 0.0f, 0.0f, 0.0f);
    auto line1 = makeLine(0.0f, 0.0f, 1.0f, 0.0f);

    float angle = planner.angle_between_lines(line0, line1);

    EXPECT_TRUE(std::isinf(angle));
}
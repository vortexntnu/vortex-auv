#include <gtest/gtest.h>
#include <Eigen/Core>
#include <vector>
#include <cmath>

#include <pool_exploration/pool_exploration.hpp>

using namespace vortex::pool_exploration;

namespace
{
LineSegment makeLine(float x0, float y0, float x1, float y1)
{
    LineSegment line;
    line.p0.x = x0;
    line.p0.y = y0;
    line.p1.x = x1;
    line.p1.y = y1;
    return line;
}

PoolExplorationPlannerConfig makeDefaultConfig()
{
    PoolExplorationPlannerConfig config{};
    config.min_dist = 0.5f;
    config.max_dist = 20.0f;
    config.angle_threshold = 0.5f;      // ~28.6 deg
    config.min_angle = 1.4f;            
    config.max_angle = 1.60f;         
    config.right_dist = 0.1f;
    config.right_wall_offset = 1.0f;
    config.far_wall_offset = 1.0f;
    return config;
}
}  // namespace

TEST(PoolExplorationPlannerTest, ProjectDroneToHorizontalLine)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(2.0f, 3.0f);
    std::pair<Eigen::Vector2f, Eigen::Vector2f> line{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(10.0f, 0.0f)
    };

    Eigen::Vector2f projection = planner.project_drone_to_line(drone_pos, line);

    EXPECT_NEAR(projection.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(projection.y(), 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, ProjectDroneToVerticalLine)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(4.0f, 2.0f);
    std::pair<Eigen::Vector2f, Eigen::Vector2f> line{
        Eigen::Vector2f(1.0f, -5.0f),
        Eigen::Vector2f(1.0f, 5.0f)
    };

    Eigen::Vector2f projection = planner.project_drone_to_line(drone_pos, line);

    EXPECT_NEAR(projection.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(projection.y(), 2.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, ProjectDroneToDegenerateLineReturnsPoint)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    Eigen::Vector2f drone_pos(4.0f, 2.0f);
    std::pair<Eigen::Vector2f, Eigen::Vector2f> line{
        Eigen::Vector2f(1.0f, 1.0f),
        Eigen::Vector2f(1.0f, 1.0f)
    };

    Eigen::Vector2f projection = planner.project_drone_to_line(drone_pos, line);

    EXPECT_NEAR(projection.x(), 1.0f, 1e-5f);
    EXPECT_NEAR(projection.y(), 1.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, LineHeadingAngleDifferenceParallel)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(5.0f, 0.0f)
    };

    float angle = planner.line_heading_angle_difference(line, 0.0f);

    EXPECT_NEAR(angle, 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, LineHeadingAngleDifferenceParallelReversedOrder)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line{
        Eigen::Vector2f(5.0f, 0.0f),
        Eigen::Vector2f(0.0f, 0.0f)
    };

    float angle = planner.line_heading_angle_difference(line, 0.0f);

    EXPECT_NEAR(angle, 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, LineHeadingAngleDifferencePerpendicular)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(0.0f, 5.0f)
    };

    float angle = planner.line_heading_angle_difference(line, 0.0f);

    EXPECT_NEAR(angle, static_cast<float>(M_PI_2), 1e-5f);
}

TEST(PoolExplorationPlannerTest, LineIntersectionFindsCorrectPoint)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line0{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(10.0f, 0.0f)
    };

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line1{
        Eigen::Vector2f(2.0f, -5.0f),
        Eigen::Vector2f(2.0f, 5.0f)
    };

    Eigen::Vector2f intersection;
    bool intersects = planner.line_intersection(line0, line1, intersection);

    EXPECT_TRUE(intersects);
    EXPECT_NEAR(intersection.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(intersection.y(), 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, LineIntersectionParallelReturnsFalse)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line0{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(10.0f, 0.0f)
    };

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line1{
        Eigen::Vector2f(0.0f, 1.0f),
        Eigen::Vector2f(10.0f, 1.0f)
    };

    Eigen::Vector2f intersection;
    bool intersects = planner.line_intersection(line0, line1, intersection);

    EXPECT_FALSE(intersects);
}

TEST(PoolExplorationPlannerTest, AngleBetweenLinesPerpendicular)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line0{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(10.0f, 0.0f)
    };

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line1{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(0.0f, 10.0f)
    };

    float angle = planner.angle_between_lines(line0, line1);

    EXPECT_NEAR(angle, static_cast<float>(M_PI_2), 1e-5f);
}

TEST(PoolExplorationPlannerTest, AngleBetweenLinesParallelReversedOrderStillZero)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line0{
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(10.0f, 0.0f)
    };

    std::pair<Eigen::Vector2f, Eigen::Vector2f> line1{
        Eigen::Vector2f(10.0f, 0.0f),
        Eigen::Vector2f(0.0f, 0.0f)
    };

    float angle = planner.angle_between_lines(line0, line1);

    EXPECT_NEAR(angle, 0.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, SelectBestCornerChoosesClosestCorner)
{
    PoolExplorationPlanner planner(makeDefaultConfig());

    CandidateCorner c0;
    c0.corner_point = Eigen::Vector2f(10.0f, 10.0f);

    CandidateCorner c1;
    c1.corner_point = Eigen::Vector2f(2.0f, 1.0f);

    CandidateCorner c2;
    c2.corner_point = Eigen::Vector2f(5.0f, 5.0f);

    std::vector<CandidateCorner> corners{c0, c1, c2};
    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    CandidateCorner best = planner.select_best_corner(corners, drone_pos);

    EXPECT_NEAR(best.corner_point.x(), 2.0f, 1e-5f);
    EXPECT_NEAR(best.corner_point.y(), 1.0f, 1e-5f);
}

TEST(PoolExplorationPlannerTest, FindValidCornerFindsOneCornerFromRightAndFarWall)
{
    PoolExplorationPlannerConfig config = makeDefaultConfig();
    config.angle_threshold = 0.6f;   // ~34 deg
    config.min_angle = 1.0f;         // romsligere nedre grense
    config.max_angle = 1.6f;         // litt over pi/2
    config.min_dist = 0.5f;
    config.max_dist = 20.0f;
    config.right_dist = 0.1f;

    PoolExplorationPlanner planner(config);

    Eigen::Vector2f drone_pos(0.0f, 0.0f);
    float drone_heading = 0.0f;

    // Matcher dagens right-definisjon i implementasjonen:
    // right = (-sin(heading), cos(heading))
    // ved heading = 0 blir dette (0, 1), altså positiv y.
    LineSegment right_wall = makeLine(-10.0f, 2.0f, 10.0f, 2.0f);
    LineSegment far_wall   = makeLine(5.0f, -10.0f, 5.0f, 10.0f);

    std::vector<LineSegment> lines{right_wall, far_wall};

    std::vector<CandidateCorner> corners =
        planner.find_valid_corner(lines, drone_pos, drone_heading);

    ASSERT_EQ(corners.size(), 1u);
    EXPECT_NEAR(corners[0].corner_point.x(), 5.0f, 1e-4f);
    EXPECT_NEAR(corners[0].corner_point.y(), 2.0f, 1e-4f);
}

TEST(PoolExplorationPlannerTest, EstimateDockingPositionOffsetsFromCorner)
{
    PoolExplorationPlannerConfig config = makeDefaultConfig();
    config.right_wall_offset = 1.0f;
    config.far_wall_offset = 2.0f;

    PoolExplorationPlanner planner(config);

    CandidateCorner corner;
    corner.right_wall = makeLine(0.0f, 2.0f, 10.0f, 2.0f);
    corner.far_wall   = makeLine(5.0f, -5.0f, 5.0f, 5.0f);
    corner.corner_point = Eigen::Vector2f(5.0f, 2.0f);

    Eigen::Vector2f drone_pos(0.0f, 0.0f);

    Eigen::Vector2f docking = planner.estimate_docking_position(corner, drone_pos);

    // Denne forventningen matcher dagens compute_inward_normal()-logikk,
    // altså normal som peker mot drone-siden av veggen.
    //
    // For right_wall y=2 med drone i (0,0), peker normalen nedover: (0,-1)
    // For far_wall x=5 med drone i (0,0), peker normalen mot venstre: (-1,0)
    //
    // docking = (5,2) + 1*(0,-1) + 2*(-1,0) = (3,1)
    EXPECT_NEAR(docking.x(), 3.0f, 1e-4f);
    EXPECT_NEAR(docking.y(), 1.0f, 1e-4f);
}
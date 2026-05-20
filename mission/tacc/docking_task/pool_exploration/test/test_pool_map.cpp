#include <gtest/gtest.h>
#include <pool_exploration/pool_exploration.hpp>
#include <vortex_msgs/msg/line_segment2_d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace vortex::pool_exploration;
/*
TEST(PoolExplorationTest, DrawLinesIdentityTransform)
{
    PoolExplorationMap map(10.0, 10.0, 1.0, "map");

    // Lag et LineSegment2DArray med én linje
    auto array_msg = std::make_shared<vortex_msgs::msg::LineSegment2DArray>();
    vortex_msgs::msg::LineSegment2D line;
    line.p0.x = 1.0;
    line.p0.y = 0.0;
    line.p1.x = 4.0;
    line.p1.y = 0.0;
    array_msg->lines.push_back(line);

    // Identitet transform
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();

    // Kjør funksjonen som tegner linjene
    map.setLineSegmentInMapFrame(array_msg, identity);

    const auto& grid = map.grid();

    // Map origin er -5,-5
    // Punkt (1,0) blir grid (6,5)
    for (int x = 6; x <= 9; ++x) {
        int index = 5 * grid.info.width + x;
        EXPECT_EQ(grid.data[index], OCCUPIED);
    }
}


TEST(PoolExplorationTest, DebugPrint)
{
    PoolExplorationMap map(10.0, 10.0, 1.0, "map");

    map.bresenhamLineAlgoritm(2, 2, 8, 6);

    map.printGridToConsole();   // 👈 HER
}

TEST(PoolExplorationTest, GridInitialization)
{
    PoolExplorationMap map(10.0, 10.0, 1.0, "map");

    const auto& grid = map.grid();

    EXPECT_EQ(grid.info.width, 10);
    EXPECT_EQ(grid.info.height, 10);
    EXPECT_EQ(grid.data.size(), 100);

    // Alle celler skal være -1 (unknown)
    for (auto cell : grid.data) {
        EXPECT_EQ(cell, -1);
    }
}
/*
TEST(PoolExplorationTest, SetGridCellInsideBounds)
{
    PoolExplorationMap map(10.0, 10.0, 1.0, "map");

    map.setGridCell(5, 5, OCCUPIED);

    const auto& grid = map.grid();
    int index = 5 * grid.info.width + 5;

    EXPECT_EQ(grid.data[index], OCCUPIED);
}

TEST(PoolExplorationTest, BresenhamHorizontalLine)
{
    PoolExplorationMap map(10.0, 10.0, 1.0, "map");

    map.bresenhamLineAlgoritm(2, 5, 7, 5);

    const auto& grid = map.grid();

    for (int x = 2; x <= 7; ++x) {
        int index = 5 * grid.info.width + x;
        EXPECT_EQ(grid.data[index], OCCUPIED);
    }
}

TEST(PoolExplorationTest, MultipleLinesIdentityTransform)
{
    // Lag et større grid, f.eks. 20x20 meter med 1m oppløsning
    PoolExplorationMap map(20.0, 20.0, 1.0, "map");

    // Lag LineSegment2DArray-melding
    auto array_msg = std::make_shared<vortex_msgs::msg::LineSegment2DArray>();

    // Legg til flere linjer
    vortex_msgs::msg::LineSegment2D line1;
    line1.p0.x = 1.0; line1.p0.y = 0.0;
    line1.p1.x = 4.0; line1.p1.y = 0.0;
    array_msg->lines.push_back(line1);

    vortex_msgs::msg::LineSegment2D line2;
    line2.p0.x = -3.0; line2.p0.y = 2.0;
    line2.p1.x = 2.0; line2.p1.y = 5.0;
    array_msg->lines.push_back(line2);

    vortex_msgs::msg::LineSegment2D line3;
    line3.p0.x = -5.0; line3.p0.y = -5.0;
    line3.p1.x = 5.0; line3.p1.y = -5.0;
    array_msg->lines.push_back(line3);

    // Identitet transform
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();

    // Tegn linjene
    map.setLineSegmentInMapFrame(array_msg, identity);

    const auto& grid = map.grid();

    // Enkel sjekk: se at start- og sluttpunkt for hver linje er satt til OCCUPIED
    auto check_cell = [&](double x, double y) {
        int gx = static_cast<int>((x - grid.info.origin.position.x) / grid.info.resolution);
        int gy = static_cast<int>((y - grid.info.origin.position.y) / grid.info.resolution);
        int index = gy * grid.info.width + gx;
        EXPECT_EQ(grid.data[index], OCCUPIED);
    };

    // Linje 1
    check_cell(1.0, 0.0);
    check_cell(4.0, 0.0);

    // Linje 2
    check_cell(-3.0, 2.0);
    check_cell(2.0, 5.0);

    // Linje 3
    check_cell(-5.0, -5.0);
    check_cell(5.0, -5.0);

    // Valgfritt: skriv grid til console for visuell inspeksjon
    map.printGridToConsole();
}


TEST(GeometryTest, ProjectPointToLine) {
    PoolExplorationMap map(10, 10, 1.0, "map");

    Eigen::Vector2f p0(0,0);
    Eigen::Vector2f p1(4,0);
    Eigen::Vector2f drone_pos(2, 3);

    Eigen::Vector2f projection = map.projectPointToLine(drone_pos, {p0,p1});

    // Prosjekter punkt på x-akse linje => y skal bli 0
    EXPECT_FLOAT_EQ(projection.x(), 2.0f);
    EXPECT_FLOAT_EQ(projection.y(), 0.0f);
}

TEST(GeometryTest, LineAngleDifference) {
    PoolExplorationMap map(10,10,1.0,"map");

    Eigen::Vector2f p0(0,0);
    Eigen::Vector2f p1(1,1);
    float heading = 0.0f; // Drone peker mot x-akse

    float diff = map.lineAngleDifference({p0,p1}, heading);

    // Linje 45 grader mot x-aksen
    EXPECT_NEAR(diff, M_PI/4, 1e-6);
}

TEST(GeometryTest, LineIntersection) {
    PoolExplorationMap map(10,10,1.0,"map");

    Eigen::Vector2f intersection;

    // Kryssende linjer
    std::pair<Eigen::Vector2f,Eigen::Vector2f> line1{{0,0},{4,4}};
    std::pair<Eigen::Vector2f,Eigen::Vector2f> line2{{0,4},{4,0}};

    bool intersects = map.lineIntersection(line1, line2, intersection);

    EXPECT_TRUE(intersects);
    EXPECT_NEAR(intersection.x(), 2.0f, 1e-6);
    EXPECT_NEAR(intersection.y(), 2.0f, 1e-6);

    // Parallelle linjer
    std::pair<Eigen::Vector2f,Eigen::Vector2f> line3{{0,0},{1,0}};
    std::pair<Eigen::Vector2f,Eigen::Vector2f> line4{{0,1},{1,1}};

    intersects = map.lineIntersection(line3, line4, intersection);
    EXPECT_FALSE(intersects);
}

TEST(GeometryTest, AngleBetweenLines) {
    PoolExplorationMap map(10,10,1.0,"map");

    std::pair<Eigen::Vector2f,Eigen::Vector2f> line1{{0,0},{1,0}};
    std::pair<Eigen::Vector2f,Eigen::Vector2f> line2{{0,0},{0,1}};

    float angle = map.angleBetweenLines(line1, line2);

    // Linjene er ortogonale
    EXPECT_NEAR(angle, M_PI/2, 1e-6);

    // Samme linje => vinkel 0
    angle = map.angleBetweenLines(line1, line1);
    EXPECT_NEAR(angle, 0.0, 1e-6);
}

*/
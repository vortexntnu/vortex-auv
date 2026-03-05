#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

//#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
//#include <algorithm> //brukes ikke av clamp?
#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <limits>
//#include <stdexcept>

static constexpr int8_t OCCUPIED = 100;

namespace vortex::pool_exploration{
// Punkt i map frame
struct Point {
    float x{};
    float y{};
};
// Lager eget linjesegment i 2d, ikke bruk vortex_msgs sin
struct LineSegment {
    Point p0;
    Point p1;
  
    // Lage Eigen til beregninger
    std::pair<Eigen::Vector2f, Eigen::Vector2f> asEigen() const {
        return { {p0.x, p0.y}, {p1.x, p1.y} };
    }
};

//linjer som oppfyller hjørnekrav
struct CandidateCorner {
    LineSegment right_wall;
    LineSegment far_wall;
    Eigen::Vector2f corner_point;
    //float score; //Må finne en måte for å vurdere beste hjørne
};

class PoolExplorationMap {
public:
    // Konstruktør
    PoolExplorationMap(
        double size_x,
        double size_y,
        double resolution,
        const std::string& frame_id);

    const nav_msgs::msg::OccupancyGrid& grid() const;

    /**
    * @brief Draws a straight line on the occupancy grid using Bresenham's algorithm.
    * Reference: https://www.youtube.com/watch?v=CceepU1vIKo
    */
          
    void insertSegmentsMapFrame(const std::vector<LineSegment>& segments);
    
    Eigen::Vector2f estimateDockingPosition(
        const CandidateCorner estimated_corner,
        float right_wall_offset,
        float far_wall_offset);

    // brukes i selectBestCorner og deretter estimateDockingPosition
    std::vector<CandidateCorner> findCorner( 
        const std::vector<LineSegment>& lines, 
        const Eigen::Vector2f& drone_pos,
        float drone_heading,
        float min_dist, float max_dist,
        float angle_threshold,
        float min_angle, float max_angle);

    // brukes i estimateDockingPosition
    CandidateCorner selectBestCorner( 
        const std::vector<CandidateCorner>& possible_corners,
        const Eigen::Vector2f& drone_position);

    //MIDLERTIDIG TEST-FUNKSJON
    // void printGridToConsole() const;
    
private:
    //Occupancy grid
    void initialize_grid();

     //brukes i insertSegmentsMapFrame
    void bresenhamLineAlgoritm(int x0, int y0, int x1, int y1);

    // brukes i bresenhamLineAlgoritm
    void setGridCell(int x, int y, int value);

    //Antar får inn drone_pos i map frame
    Eigen::Vector2f projectPointToLine(
        const Eigen::Vector2f& drone_pos, // punktet som projiseres
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line);

    float lineAngleDifference(
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, 
        const float& drone_heading);

    //uavhengig av at det er segment
    bool lineIntersection( 
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1,
        Eigen::Vector2f& intersection_coordinates);
    
    float angleBetweenLines(
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1);

    Eigen::Vector2f computeNormal(const LineSegment& line);

   
//private:

    //Parameters for Occupancy grid
    nav_msgs::msg::OccupancyGrid grid_;
    double size_x_;
    double size_y_;
    double resolution_;

    
};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP




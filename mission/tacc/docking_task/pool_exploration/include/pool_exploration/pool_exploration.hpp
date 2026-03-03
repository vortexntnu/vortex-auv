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

    // Konverter fra ROS LineSegment3D MÅ ENDRE OG LEGGE I ROSDELEN
/*    LineSegment(const vortex_msgs::msg::LineSegment3D& msg)
        : p0{static_cast<float>(msg.p0.x), static_cast<float>(msg.p0.y)}, 
          p1{static_cast<float>(msg.p1.x), static_cast<float>(msg.p1.y)} {}

*/    
    // Lage Eigen til beregninger
    std::pair<Eigen::Vector2f, Eigen::Vector2f> asEigen() const {
        return { {p0.x, p0.y}, {p1.x, p1.y} };
    }
};

struct CandidateCorner {
    LineSegment right_wall;
    LineSegment far_wall;
    Eigen::Vector2f corner_point;
    //float score; //Må finne en måte for å vurdere beste hjørne
};

class PoolExplorationMap {
public:
    PoolExplorationMap( double size_x,
                        double size_y,
                        double resolution,
                        const std::string& frame_id);

    const nav_msgs::msg::OccupancyGrid& grid() const;
   

    /**
    * @brief Draws a straight line on the occupancy grid using Bresenham's algorithm.
    * Reference: https://www.youtube.com/watch?v=CceepU1vIKo
    */
    void bresenhamLineAlgoritm(int x0, int y0, int x1, int y1);
    
    //ENDRE INPUT PÅ DENNE (SJEKK AT _ROS oppfyller samme som denne gjorde)
    //void setLineSegmentInMapFrame(const vortex_msgs::msg::LineSegment3DArray::SharedPtr msgs,
    //                                const Eigen::Matrix4f& map_to_odom_tf);
                        
    void insertSegmentsMapFrame(const std::vector<LineSegment>& segments);
    //ENDRET
    std::vector<CandidateCorner> findCorner( const std::vector<LineSegment>& lines, 
                                             const Eigen::Vector2f& drone_pos,
                                             float drone_heading,
                                             float min_dist, float max_dist,
                                             float angle_threshold,
                                             float min_angle, float max_angle);

    //MIDLERTIDIG TEST-FUNKSJON
    void printGridToConsole() const;
    

private:
    //Occupancy grid
    void initialize_grid();
    void setGridCell(int x, int y, int value);

    Eigen::Vector2f projectPointToLine( const Eigen::Vector2f& drone_pos, // punktet som projiseres
                                        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line);

    float lineAngleDifference( const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, 
                               const float& drone_heading);

    bool lineIntersection( const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
                                            const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1,
                                            Eigen::Vector2f& intersection_coordinates);
    
    float angleBetweenLines(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
                            const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1);

    int computeScore( const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
                      const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1,
                      const Eigen::Vector2f& intersection);

    CandidateCorner selectBestCorner( const std::vector<CandidateCorner>& possible_corners,
                                      const Eigen::Vector2f& drone_position);

    Eigen::Vector2f computeNormal(const LineSegment& line);

    Eigen::Vector2f estimateDockingPosition(
        const CandidateCorner estimated_corner,
        float right_wall_offset,
        float far_wall_offset);
//private:

    //Parameters for Occupancy grid
    nav_msgs::msg::OccupancyGrid grid_;
    double size_x_;
    double size_y_;
    double resolution_;

    
};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP




#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

//#include <geometry_msgs/msg/detail/point__struct.hpp>
//#include <cstdint>
#include <nav_msgs/msg/occupancy_grid.hpp>
//#include <string>
#include <vector>
#include <utility>
#include <cmath>
//#include <algorithm> //brukes ikke av clamp?
#include <Eigen/Core>
#include <Eigen/Geometry>

//#include <limits>
//#include <stdexcept>

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
};

struct PoolExplorationPlannerConfig {
    // fyll inn config parametere?
    // må declare
    float min_dist; 
    float max_dist;
    float angle_threshold;
    float min_angle;
    float max_angle;

    float right_dist;

    float right_wall_offset;
    float far_wall_offset;
};

class PoolExplorationPlanner {
public:
    PoolExplorationPlanner(
        const PoolExplorationPlannerConfig& config);

    // bruker compute_normal til å finne normalt på veggene
    // henter ut intersection fra estimated_corner og bruker offsetene
    Eigen::Vector2f estimate_docking_position(
        const CandidateCorner& estimated_corner,
        const Eigen::Vector2f& drone_pos);

    // brukes i selectBestCorner og deretter estimateDockingPosition
    //Funksjon som finner hjørner basert på krav om vinkler
    // Denne må ta inn linjer i odom frame for at projec\ksjonkravene oppfylles riktig

    // 1. finner separate linjer som hver oppfyller kravene
    // Projiserer drone på linje og måle distanser
    // For hver linje i linjene sjekker den om oppfyller vinkel og distansekrav fra drone til projeksjon
    // 2. bruke lineintersection til å sjekke om to linjer skjærer hverandre og lagrer koordinatene
    // hvis krysser hverandre, sjekk om vinkel oppfyller krav og lagrer i potential_corners

    // dersom vil finne venstre i stedet for høyre: endre krav for verdi sjekket
    std::vector<CandidateCorner> find_valid_corner( 
        const std::vector<LineSegment>& lines, 
        const Eigen::Vector2f& drone_pos, // sjekk at får inn pos og heading riktig?
        float drone_heading);

    // brukes i estimateDockingPosition
    // funksjon som velger ut de to beste linjene som er hjørnet nærmest drone
    // Setter min_dist til størst mulige float først og vil oppdateres ettersom sammenligner linjeavstandene
    // setter best_corner først til det første i vectoren
    // velger det hjørnet som er nærmest dronen
    CandidateCorner select_best_corner( 
        const std::vector<CandidateCorner>& possible_corners,
        const Eigen::Vector2f& drone_pos);

    // TO DO: Lage funksjon so fjerner doble linjer?

    //MIDLERTIDIG TEST-FUNKSJON
    // void printGridToConsole() const;
    
private:
   
    //Antar får inn drone_pos i map frame
    // sjekker at linje ikke er et punkt
    // Bruker projeksjonsformelen
    Eigen::Vector2f project_drone_to_line(
        const Eigen::Vector2f& drone_pos, // punktet som projiseres
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line);

    // Returns the smallest absolute angle between a line segment and the drone heading (0 to π).
    // Sjekke logikken her en gang til TO DO
    float line_heading_angle_difference(
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, 
        float drone_heading);

    // Calculates the intersection of two infinite lines defined by point pairs and stores the result in `intersection`.
    // Returns false if the lines are parallel.
    bool line_intersection( 
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1,
        Eigen::Vector2f& intersection_coordinates);
    
    // Computes the angle between two line segments in radians.
    float angle_between_lines(
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
        const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1);

    Eigen::Vector2f compute_inward_normal(const LineSegment& line, const Eigen::Vector2f& drone_pos);

    PoolExplorationPlannerConfig config_;

    // LOGIKK FOR GRIDDET //
# if 0
public:
 // Konstruktør
    PoolExplorationPlanner(
        double size_x,
        double size_y,
        double resolution,
        const std::string& frame_id);

    const nav_msgs::msg::OccupancyGrid& grid() const;

  /**
    * @brief Draws a straight line on the occupancy grid using Bresenham's algorithm.
    * Reference: https://www.youtube.com/watch?v=CceepU1vIKo
    */
    // denne kalles fra ros noden
    // fra map til celle (forklar mer hvordan)
    void insert_line_in_grid(const std::vector<LineSegment>& segments);
    
private:
    //Occupancy grid
    // Set all cells unknown, 
    // origin in center of map with no rotation
    void initialize_grid();

     //brukes i insertSegmentsMapFrame
     // bresenham line algoritm (referanse)
    void bresenham_line_algoritm(int x0, int y0, int x1, int y1);

    // brukes i bresenhamLineAlgoritm
    // sjekke at koordinatene er innenfor mappet
    void set_grid_cell(int x, int y, int value);

    nav_msgs::msg::OccupancyGrid grid_;
    std::string frame_id;
    double size_x_;
    double size_y_;
    double resolution_;

    int8_t occupied_cell_;  
    int8_t unknown_cell_;
#endif

};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP




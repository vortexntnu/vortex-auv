#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vortex/utils/types.hpp>

namespace vortex::pool_exploration{

struct CornerEstimate {
    vortex::utils::types::LineSegment2D right_wall;
    vortex::utils::types::LineSegment2D far_wall;
    Eigen::Vector2f corner_point;
};

enum class WallClassification {
    RightWall,
    LeftWall,
    FrontWall,
    Rejected
};

struct PoolExplorationPlannerConfig {
    float min_wall_distance_m; 
    float max_wall_distance_m;

    float far_wall_heading_angle_threshold; //gammel kode FJERN TO DO
    float parallel_heading_angle_threshold_rad;
    float perpendicular_heading_angle_threshold_rad;

    float min_corner_angle_rad;
    float max_corner_angle_rad;

    float right_dist; // gammel kode, FJERN TO DO
    float right_wall_max_y_m;
    float far_wall_min_x_m;
    float right_wall_offset_m;
    float far_wall_offset_m;
    int choose_right_corner;
};

class PoolExplorationPlanner {
public:
    PoolExplorationPlanner(
        const PoolExplorationPlannerConfig& config);

    // bruker compute_normal til å finne normalt på veggene
    // henter ut intersection fra estimated_corner og bruker offsetene
    Eigen::Vector2f estimate_docking_position(
        const CornerEstimate& estimated_corner,
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
    std::vector<CornerEstimate> find_corner_estimates( 
        const std::vector<vortex::utils::types::LineSegment2D>& lines, 
        const Eigen::Vector2f& drone_pos, // sjekk at får inn pos og heading riktig?
        float drone_heading);

    // brukes i estimateDockingPosition
    // funksjon som velger ut de to beste linjene som er hjørnet nærmest drone
    // Setter min_dist til størst mulige float først og vil oppdateres ettersom sammenligner linjeavstandene
    // setter best_corner først til det første i vectoren
    // velger det hjørnet som er nærmest dronen
    CornerEstimate select_best_corner( 
        const std::vector<CornerEstimate>& possible_corners,
        const Eigen::Vector2f& drone_pos);

    // TO DO: Lage funksjon so fjerner doble linjer?

    //MIDLERTIDIG TEST-FUNKSJON
    // void printGridToConsole() const;
    
//private:
   
    //Antar får inn drone_pos i map frame
    // sjekker at linje ikke er et punkt
    // Bruker projeksjonsformelen
    Eigen::Vector2f project_point_onto_line(
        const Eigen::Vector2f& drone_pos, // punktet som projiseres
        const vortex::utils::types::LineSegment2D& line);

    // Returns the smallest absolute angle between a line segment and the drone heading (0 to π).
    // Sjekke logikken her en gang til TO DO
    float angle_between_line_and_heading(
        const vortex::utils::types::LineSegment2D& line, 
        float drone_heading);

    // Calculates the intersection of two infinite lines defined by point pairs and stores the result in `intersection`.
    // Returns false if the lines are parallel.
    bool compute_line_intersection( 
        const vortex::utils::types::LineSegment2D& line0,
        const vortex::utils::types::LineSegment2D& line1,
        Eigen::Vector2f& intersection_coordinates);
    
    // Computes the angle between two line segments in radians.
    float angle_between_lines(
        const vortex::utils::types::LineSegment2D& line0,
        const vortex::utils::types::LineSegment2D& line1);

    Eigen::Vector2f compute_normal_towards_point(const vortex::utils::types::LineSegment2D& line, const Eigen::Vector2f& drone_pos);

    WallClassification classify_wall(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos,
    float drone_heading);

    PoolExplorationPlannerConfig config_;

};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP




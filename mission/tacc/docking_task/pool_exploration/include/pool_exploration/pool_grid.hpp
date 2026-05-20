#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vortex/utils/types.hpp>

#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vortex::pool_exploration{
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
    // denne kalles fra ros noden
    // fra map til celle (forklar mer hvordan)
    void insert_line_in_grid(const std::vector<vortex::utils::types::LineSegment2D>& segments); // NB: har endret til utils type linesegment, har ikke asEigen funksjonen
    
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

};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP
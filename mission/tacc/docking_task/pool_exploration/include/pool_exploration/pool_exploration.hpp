#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>

namespace vortex::pool_exploration{

class PoolExplorationMap {
public:
    PoolExplorationMap( double size_x,
                        double size_y,
                        double resolution,
                        const std::string& frame_id);

    const nav_msgs::msg::OccupancyGrid& grid() const;

private:
    //Occupancy grid
    void initialize_grid();

    //Parameters for Occupancy grid
    nav_msgs::msg::OccupancyGrid grid_;
    double size_x_;
    double size_y_;
    double resolution_;
};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP








/*
DETTE SKAL I .cpp FILA:
initialize_mapper_params()*/

//Lage structs for occupancy grid?
/*
    size_x
    size_y
    resolution
    dilation
*/

//struct med parametere?
/*
    min_dist
    max_dist
    angle_treshold
    min_angle
    max_angle
    far_wall_offset
    left_wall_offset

*/


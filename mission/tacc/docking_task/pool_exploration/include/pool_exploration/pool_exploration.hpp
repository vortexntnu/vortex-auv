#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vortex::pool_exploration{

//start og sluttverdi for linjen som hentes inn
struct LineSegment {
    Eigen::Vector2f p1;
    Eigen::Vector2f p2;
};

class PoolExplorationMap {
public:
    PoolExplorationMap( double size_x,
                        double size_y,
                        double resolution,
                        const std::string& frame_id);

    const nav_msgs::msg::OccupancyGrid& grid() const;
    static LineSegment rhoThetaToSegment(double rho, double theta, float length); //Bestem om den er static/vanlig medlem eller utenfor senere
    void setLineSegmentInMapFrame(const LineSegment& seg,
                                  const Eigen::Matrix4f& map_to_odom_tf,
                                  int8_t value);

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


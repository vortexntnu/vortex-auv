#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vortex_msgs/msg/line_segment2_d_array.hpp>

static constexpr int8_t OCCUPIED = 100;


namespace vortex::pool_exploration{

class PoolExplorationMap {
public:
    PoolExplorationMap( double size_x,
                        double size_y,
                        double resolution,
                        const std::string& frame_id);

    const nav_msgs::msg::OccupancyGrid& grid() const;
   
    void setGridCell(int x, int y, int value);
    void bresenhamLineAlgoritm(int x0, int y0, int x1, int y1);
    
    void setLineSegmentInMapFrame(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msgs,
                                    const Eigen::Matrix4f& map_to_odom_tf);
    
    //MIDLERTIDIG TEST-FUNKSJON
    void printGridToConsole() const;

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




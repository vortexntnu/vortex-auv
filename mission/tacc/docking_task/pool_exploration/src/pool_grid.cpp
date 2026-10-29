#include <cstdlib>
#include <pool_exploration/pool_grid.hpp>
#include <vortex/utils/types.hpp>
#include <spdlog/spdlog.h> // til testing

#include <Eigen/src/Core/Matrix.h>

namespace vortex::pool_exploration{

// Lage egen config til griddet TO DO
PoolExplorationMap::PoolExplorationMap(
    double size_x,
    double size_y,
    double resolution,
    const std::string& frame_id)
    : size_x_(size_x),
      size_y_(size_y),
      resolution_(resolution) {
    grid_.header.frame_id = frame_id;
    initialize_grid();
}

void PoolExplorationMap::initialize_grid() {
    int width  = static_cast<int>(size_x_ / resolution_);
    int height = static_cast<int>(size_y_ / resolution_);

    grid_.info.resolution = resolution_;
    grid_.info.width      = width;
    grid_.info.height     = height;

    grid_.info.origin.position.x = -size_x_ / 2.0;
    grid_.info.origin.position.y = -size_y_ / 2.0;
    grid_.info.origin.orientation.w = 1.0; 

    grid_.data.assign(width * height, unknown_cell_);
}

const nav_msgs::msg::OccupancyGrid& PoolExplorationMap::grid() const {
    return grid_;
}

void PoolExplorationMap::insert_line_in_grid(
    const std::vector<utils::types::LineSegment2D>& segments) {
    for (const auto& seg : segments) {

        int x0 = static_cast<int>((seg.p0.x - grid_.info.origin.position.x) / grid_.info.resolution);
        int y0 = static_cast<int>((seg.p0.y - grid_.info.origin.position.y) / grid_.info.resolution);
        int x1 = static_cast<int>((seg.p1.x - grid_.info.origin.position.x) / grid_.info.resolution);
        int y1 = static_cast<int>((seg.p1.y - grid_.info.origin.position.y) / grid_.info.resolution);

        bresenham_line_algoritm(x0, y0, x1, y1);
    }
}

void PoolExplorationMap::bresenham_line_algoritm(
    int x0, int y0, int x1, int y1) {
    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (!steep){
        if (x0 > x1){
        std::swap(x0, x1);
        std::swap(y0,y1);
        }

        int dx = x1 - x0;
        int dy = y1 - y0;

        int dir = (dy <0) ? -1 : 1;
        dy = std::abs(dy);

        int y = y0;
        int p = 2 * dy - dx;

        for (int i = 0; i <= dx; i++) {
            set_grid_cell(x0 + i, y, occupied_cell_); // må declare occupied_cell_ i ros senere

            if (p >= 0) {
                y += dir;
                p -= 2 * dx;
            }
            p += 2 * dy;
        }
    } else {
        if (y0 > y1){
        std::swap(x0, x1);
        std::swap(y0,y1);
        }

        int dx = x1 - x0;
        int dy = y1 - y0;

        int dir = (dx <0) ? -1 : 1;
        dx = std::abs(dx);

        int x = x0;
        int p = 2 * dx - dy;

        for (int i = 0; i <= dy; i++) {
            set_grid_cell(x, y0 + i, occupied_cell_);

            if (p >= 0) {
                x += dir;
                p -= 2 * dy;
            }
            p += 2 * dx;
        }
    }
}

void PoolExplorationMap::set_grid_cell(
    int x, int y, int value) {
    if (x < 0 || x >= static_cast<int>(grid_.info.width) ||
        y < 0 || y >= static_cast<int>(grid_.info.height)) {
        return;
    }
    int index = y * grid_.info.width + x;
    grid_.data[index] = value;
}
  

/*
//FUNKSJON BRUKT TIL TESTING, FJERNE SENERE:))?
void PoolExplorationPlanner::printGridToConsole() const{
    for (int y = grid_.info.height - 1; y >= 0; --y) {
        for (int x = 0; x < grid_.info.width; ++x) {

            int index = y * grid_.info.width + x;

            if (grid_.data[index] == OCCUPIED)
                std::cout << "X ";
            else if (grid_.data[index] == -1)
                std::cout << ". ";
            else
                std::cout << "? ";
        }
        std::cout << ::std::endl;
    }
}
*/

}  // namespace vortex::pool_exploration

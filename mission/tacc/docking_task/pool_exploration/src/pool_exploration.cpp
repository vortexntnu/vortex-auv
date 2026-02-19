#include <pool_exploration/pool_exploration.hpp>

namespace vortex::pool_exploration{

PoolExplorationMap::PoolExplorationMap(
    double size_x,
    double size_y,
    double resolution,
    const std::string& frame_id)
    : size_x_(size_x),
      size_y_(size_y),
      resolution_(resolution)
{
    //send in map to constructor
    grid_.header.frame_id = frame_id;
    initialize_grid();
}

void PoolExplorationMap::initialize_grid() {
    int width  = static_cast<int>(size_x_ / resolution_);
    int height = static_cast<int>(size_y_ / resolution_);

    grid_.info.resolution = resolution_;
    grid_.info.width      = width;
    grid_.info.height     = height;

    //center of map, no rotation
    grid_.info.origin.position.x = -size_x_ / 2.0;
    grid_.info.origin.position.y = -size_y_ / 2.0;
    grid_.info.origin.orientation.w = 1.0; 

    //Set all cells unknown
    grid_.data.assign(width * height, -1);

}

const nav_msgs::msg::OccupancyGrid& PoolExplorationMap::grid() const {
    return grid_;
}

//Får ikke inn parametrisering av linjene som rho, theta likevel (?)
/*
LineSegment PoolExplorationMap::rhoThetaToSegment(double rho, double theta, float length) {
    // Finn punkt på linjen nærmest origo
    double x0 = rho * cos(theta);
    double y0 = rho * sin(theta);

    // Retningsvektor for linjen
    double dx = -sin(theta);
    double dy = cos(theta);

    LineSegment seg;
    seg.p1 = Eigen::Vector2f(x0 + (length)/2*dx, y0 + (length)/2*dy);
    seg.p2 = Eigen::Vector2f(x0 - (length)/2*dx, y0 - (length)/2*dy);
    return seg;
}

*/


//FUNKSJON SOM TAR FLERE LINJER
/*
void PoolExplorationMap::drawLines(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg){
    uint32_t num_lines =  msg->lines.size();

    }
*/


void PoolExplorationMap::setLineSegmentInMapFrame(
    const vortex_msgs::msg::LineSegment2D::SharedPtr msg,
    const Eigen::Matrix4f& map_to_odom_tf)
{
    // 1. Transform fra odom → map
    // Konvertere fra 2d vektor til 4d for å matche transformasjonen
    Eigen::Vector4f p0_4d(msg->p0.x, msg->p0.y, 0.0f, 1.0f);
    Eigen::Vector4f p1_4d(msg->p1.x, msg->p1.y, 0.0f, 1.0f);
    Eigen::Vector4f p0_map = map_to_odom_tf.inverse() * p0_4d;
    Eigen::Vector4f p1_map = map_to_odom_tf.inverse() * p1_4d;

    //2. Transform fra map coordinate -> grid index
    int x0 = (p0_map.x() - grid_.info.origin.position.x) / grid_.info.resolution; 
    int y0 = (p0_map.y() - grid_.info.origin.position.y) / grid_.info.resolution; 
    int x1 = (p1_map.x() - grid_.info.origin.position.x) / grid_.info.resolution; 
    int y1 = (p1_map.y() - grid_.info.origin.position.y) / grid_.info.resolution; 

    //3. Bresenham line algoritm
    bresenhamLineAlgoritm(x0, y0, x1, y1);
}

void PoolExplorationMap::setGridCell(int x, int y, int value)
{
    // Sjekk at koordinatene er innenfor kartet
    if (x < 0 || x >= static_cast<int>(grid_.info.width) ||
        y < 0 || y >= static_cast<int>(grid_.info.height)) {
        return;
    }

    int index = y * grid_.info.width + x;

    grid_.data[index] = value;
}


void PoolExplorationMap::bresenhamLineAlgoritm(int x0, int y0, int x1, int y1){

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
            setGridCell(x0 + i, y, OCCUPIED);

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
            setGridCell(x, y0 + i, OCCUPIED);

            if (p >= 0) {
                x += dir;
                p -= 2 * dy;
            }
            p += 2 * dx;
        }
    }
}



}  // namespace vortex::pool_exploration

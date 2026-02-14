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

//Får inn parametrisering av linjene som rho, theta (?)
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

void PoolExplorationMap::setLineSegmentInMapFrame(
    const LineSegment& seg,
    const Eigen::Matrix4f& map_to_odom_tf,
    int8_t value) // value is feks 100 if occupied, to be decided
{
    // 1. Transform fra odom → map
    // Konvertere fra 2d vektor til 4d for å matche transformasjonen
    Eigen::Vector4f p1_4d(seg.p1.x(), seg.p1.y(), 0.0f, 1.0f);
    Eigen::Vector4f p2_4d(seg.p2.x(), seg.p2.y(), 0.0f, 1.0f);

    Eigen::Vector4f p1_map = map_to_odom_tf.inverse() * p1_4d;
    Eigen::Vector4f p2_map = map_to_odom_tf.inverse() * p2_4d;

    // 2. Tegn linjen i gridet med enkel interpolasjon
    // USE BRESENHAM LINE ALGORITHM
    //IMPLEMENT HERE 
    // :-)
}



}  // namespace vortex::pool_exploration


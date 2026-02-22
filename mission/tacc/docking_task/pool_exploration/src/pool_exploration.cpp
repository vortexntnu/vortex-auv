#include <pool_exploration/pool_exploration.hpp>

#include <iostream>

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

void PoolExplorationMap::setLineSegmentInMapFrame(
    const vortex_msgs::msg::LineSegment2DArray::SharedPtr msgs,
    const Eigen::Matrix4f& map_to_odom_tf)
{
    for (const auto &line : msgs->lines) {
        // 1. Transform fra odom → map
        // Konvertere fra 2d vektor til 4d for å matche transformasjonen
        Eigen::Vector4f p0_4d(line.p0.x, line.p0.y, 0.0f, 1.0f);
        Eigen::Vector4f p1_4d(line.p1.x, line.p1.y, 0.0f, 1.0f);
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


std::vector<CandidateCorner> PoolExplorationMap::findCorner(const vortex_msgs::msg::LineSegment2DArray::SharedPtr msg, 
                                                            const Eigen::Vector2f& drone_pos,
                                                            float drone_heading,
                                                            float min_dist, float max_dist,
                                                            float angle_threshold,
                                                            float min_angle, float max_angle) {
    std::vector<LineSegment> right_candidates;
    std::vector<LineSegment> far_candidates;

    // Finne mulige linjer
    for (auto& msg_line : msg->lines) {
        LineSegment line(msg_line);  // konverter her
        auto projection = projectPointToLine(drone_pos, line.asEigen());
        float dist = (projection - drone_pos).norm(); //Lengden til vektoren
        float angle = lineAngleDifference(line.asEigen(), drone_heading);

        if (projection.y() < 0 && dist >= min_dist && dist <= max_dist && angle < angle_threshold)
            right_candidates.push_back(line);
        if (projection.x() > 0 && dist >= min_dist && dist <= max_dist && angle > angle_threshold)
            far_candidates.push_back(line);
    }

    // Finne gyldige skjæringspunkt
    std::vector<CandidateCorner> corners;
    for (auto& r : right_candidates) {
        for (auto& f : far_candidates) {
            Eigen::Vector2f intersection_coordinates{};
            bool intersect = lineIntersection(r.asEigen(), f.asEigen(), intersection_coordinates);
            if (intersect){
                float wall_angle = angleBetweenLines(r.asEigen(), f.asEigen());
                if (wall_angle >= min_angle && wall_angle <= max_angle) {
                    corners.push_back({r, f, intersection_coordinates});
                }
            }
        }
    } 
    return corners;   
}
    

Eigen::Vector2f PoolExplorationMap::projectPointToLine( const Eigen::Vector2f& drone_pos,
                                    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line) {
    const Eigen::Vector2f& p0 = line.first;
    const Eigen::Vector2f& p00 = line.second;

    Eigen::Vector2f dir_vec = p00 - p0;
    float t = (drone_pos - p0).dot(dir_vec) / dir_vec.dot(dir_vec);
    return p0 + t * dir_vec;
    }

float PoolExplorationMap::lineAngleDifference( const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, const float& drone_heading){
    float line_angle = atan2(line.second.y() - line.first.y(), line.second.x() - line.first.x());
    float diff       = std::fmod(line_angle - drone_heading, 2*M_PI);
    
    if (diff < -M_PI) diff += 2*M_PI;
    if (diff >  M_PI) diff -= 2*M_PI;

    return std::abs(diff);
}

 bool PoolExplorationMap::lineIntersection( const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
                                            const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1,
                                            Eigen::Vector2f& intersection) {
    const Eigen::Vector2f p00 = line0.first;
    const Eigen::Vector2f p01 = line0.second;
    const Eigen::Vector2f p10 = line1.first;
    const Eigen::Vector2f p11 = line1.second;

    float denom = (p00.x()-p01.x())*(p10.y()-p11.y()) - (p00.y()-p01.y())*(p10.x()-p11.x());

    if (std::abs(denom) < 1e-6) 
        return false; // parallelle linjer

    intersection.x() = ((p00.x()*p01.y()-p00.y()*p01.x())*(p10.x()-p11.x()) -
                        (p00.x()-p01.x())*(p10.x()*p11.y()-p10.y()*p11.x()))/denom;
    intersection.y() = ((p00.x()*p01.y()-p00.y()*p01.x())*(p10.y()-p11.y()) -
                        (p00.y()-p01.y())*(p10.x()*p11.y()-p10.y()*p11.x()))/denom;
    return true;
}

float PoolExplorationMap::angleBetweenLines(const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
                                            const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1) {
    Eigen::Vector2f v0 = (line0.second - line0.first).normalized();
    Eigen::Vector2f v1 = (line1.second - line1.first).normalized();

    return std::acos(v0.dot(v1));     
}

/*
int PoolExplorationMap::computeScore( const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
                                      const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1,
                                      const Eigen::Vector2f& intersection){
    //TO DO: Prioriteze the lines
    return 0;
}
*/

//FUNKSJON BRUKT TIL TESTING, FJERNE SENERE:))?
void PoolExplorationMap::printGridToConsole() const{
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

}  // namespace vortex::pool_exploration

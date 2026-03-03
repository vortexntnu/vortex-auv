#include <pool_exploration/pool_exploration.hpp>

//#include <iostream> Brukt til testinga
#include <vector>

namespace vortex::pool_exploration{

PoolExplorationMap::PoolExplorationMap(
    double size_x,
    double size_y,
    double resolution,
    const std::string& frame_id)
    : size_x_(size_x),
      size_y_(size_y),
      resolution_(resolution) {
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

void PoolExplorationMap::insertSegmentsMapFrame(
    const std::vector<LineSegment>& segments)
{
    for (const LineSegment& seg : segments) {

        // map (meter) -> grid index
        int x0 = static_cast<int>((seg.p0.x - grid_.info.origin.position.x) / grid_.info.resolution);
        int y0 = static_cast<int>((seg.p0.y - grid_.info.origin.position.y) / grid_.info.resolution);
        int x1 = static_cast<int>((seg.p1.x - grid_.info.origin.position.x) / grid_.info.resolution);
        int y1 = static_cast<int>((seg.p1.y - grid_.info.origin.position.y) / grid_.info.resolution);

        bresenhamLineAlgoritm(x0, y0, x1, y1);
    }
}

void PoolExplorationMap::setGridCell(
    int x, 
    int y, 
    int value) {
    // Sjekk at koordinatene er innenfor kartet
    if (x < 0 || x >= static_cast<int>(grid_.info.width) ||
        y < 0 || y >= static_cast<int>(grid_.info.height)) {
        return;
    }

    int index = y * grid_.info.width + x;

    grid_.data[index] = value;
}


void PoolExplorationMap::bresenhamLineAlgoritm(
    int x0, 
    int y0, 
    int x1, 
    int y1) {
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

//Funksjon som finner hjørner basert på krav om vinkler
std::vector<CandidateCorner> PoolExplorationMap::findCorner(
    const std::vector<LineSegment>& lines,
    const Eigen::Vector2f& drone_pos,
    float drone_heading,
    float min_dist, 
    float max_dist,
    float angle_threshold,
    float min_angle, 
    float max_angle) {
    std::vector<LineSegment> right_candidates;
    std::vector<LineSegment> far_candidates;

    // Finne mulige linjer
    for (const auto& line : lines) {
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
    

Eigen::Vector2f PoolExplorationMap::projectPointToLine( 
    const Eigen::Vector2f& drone_pos,
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line) {
    const Eigen::Vector2f& p0 = line.first;
    const Eigen::Vector2f& p1 = line.second;

    Eigen::Vector2f dir_vec = p1 - p0;
    float len_sq = dir_vec.squaredNorm();

    // Unngå deling på null hvis segmentet er degenerert
    if (len_sq < 1e-6f) {
        return p0;  // segmentet er egentlig bare ett punkt, endre dette??
    }
    //prosjeksjon formel
    float t = (drone_pos - p0).dot(dir_vec) / len_sq;

    // projeksjon må ligge på segmentet
    t = std::clamp(t, 0.0f, 1.0f);

    return p0 + t * dir_vec;
}


float PoolExplorationMap::lineAngleDifference( 
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, 
    const float& drone_heading) {
    float line_angle = atan2(line.second.y() - line.first.y(), line.second.x() - line.first.x());
    float diff       = std::fmod(line_angle - drone_heading, 2*M_PI);
    
    if (diff < -M_PI) diff += 2*M_PI;
    if (diff >  M_PI) diff -= 2*M_PI;

    return std::abs(diff);
}

 bool PoolExplorationMap::lineIntersection( 
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
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

float PoolExplorationMap::angleBetweenLines(
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
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

//Lage en funksjon som faktisk velger ut de to beste linjene som er hjørnet
//returnerer estimated corner som er nærmest dronen (foreløpig, gjøre bedre estimat?)
CandidateCorner PoolExplorationMap::selectBestCorner(
    const std::vector<CandidateCorner>& possible_corners,
    const Eigen::Vector2f& drone_position)
{
    //finne bedre unnntakshåndtering?
    if (possible_corners.empty()) {
        throw std::runtime_error("No candidate corners available");
    }

    // Setter til størst mulige float og vil oppdateres etterpå
    float min_distance = std::numeric_limits<float>::max();
    CandidateCorner best_corner = possible_corners.front();

    for (const auto& corner : possible_corners) {
        float distance = (corner.corner_point - drone_position).squaredNorm();

        if (distance < min_distance) {
            min_distance = distance;
            best_corner = corner;
        }
    }

    return best_corner;
}


Eigen::Vector2f PoolExplorationMap::computeNormal(
    const LineSegment& line) {
    Eigen::Vector2f p0 = line.asEigen().first;
    Eigen::Vector2f p1 = line.asEigen().second;

    Eigen::Vector2f dir = (p1 - p0).normalized();
    Eigen::Vector2f n(-dir.y(), dir.x());

    return n.normalized();
}

Eigen::Vector2f PoolExplorationMap::estimateDockingPosition(
    const CandidateCorner estimated_corner,
    float right_wall_offset,
    float far_wall_offset) {
    //Linje normalt på
    Eigen::Vector2f right_normal = computeNormal(estimated_corner.right_wall);
    Eigen::Vector2f far_normal   = computeNormal(estimated_corner.far_wall);

    //må hente intersection som ligger lagret i corner
    Eigen::Vector2f docking_estimate = estimated_corner.corner_point
        + right_normal * right_wall_offset
        + far_normal   * far_wall_offset;

    return docking_estimate;
}

/*
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
*/
}  // namespace vortex::pool_exploration

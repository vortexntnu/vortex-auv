#include <pool_exploration/pool_exploration.hpp>

#include <vector>
#include <Eigen/src/Core/Matrix.h>

namespace vortex::pool_exploration{

PoolExplorationPlanner::PoolExplorationPlanner(const PoolExplorationPlannerConfig& config)
    : config_(config) {}

Eigen::Vector2f PoolExplorationPlanner::estimate_docking_position(
    const CandidateCorner& estimated_corner,
    const Eigen::Vector2f& drone_pos) {
    Eigen::Vector2f right_normal = compute_inward_normal(estimated_corner.right_wall, drone_pos);
    Eigen::Vector2f far_normal   = compute_inward_normal(estimated_corner.far_wall, drone_pos);

    Eigen::Vector2f docking_estimate = estimated_corner.corner_point
        + right_normal * config_.right_wall_offset
        + far_normal   * config_.far_wall_offset;

    return docking_estimate;
}

CandidateCorner PoolExplorationPlanner::select_best_corner(
    const std::vector<CandidateCorner>& possible_corners,
    const Eigen::Vector2f& drone_pos) {
    if (possible_corners.empty()) {
        throw std::runtime_error("No candidate corners available");  //finne bedre unnntakshåndtering?
    }

    float min_distance = std::numeric_limits<float>::max();
    CandidateCorner best_corner = possible_corners.front();

    for (const auto& corner : possible_corners) {
        float distance = (corner.corner_point - drone_pos).squaredNorm();

        if (distance < min_distance) {
            min_distance = distance;
            best_corner = corner;
        }
    }
    return best_corner;
}

std::vector<CandidateCorner> PoolExplorationPlanner::find_valid_corner(
    const std::vector<LineSegment>& lines,
    const Eigen::Vector2f& drone_pos, 
    float drone_heading) {

    std::vector<LineSegment> right_candidates;
    std::vector<LineSegment> far_candidates;
    std::vector<CandidateCorner> potential_corners;

    // Finner ut hva som er foran ogsånn? (?)
    Eigen::Vector2f forward(std::cos(drone_heading), std::sin(drone_heading));
    Eigen::Vector2f right(-std::sin(drone_heading), std::cos(drone_heading));

    for (const auto& line : lines) {
        Eigen::Vector2f projection = project_drone_to_line(drone_pos, line.asEigen());
        Eigen::Vector2f rel = projection - drone_pos;

        float dist = rel.norm(); 
        float angle = line_heading_angle_difference(line.asEigen(), drone_heading);

        float forward_dist = rel.dot(forward);
        float right_dist = rel.dot(right);

        if (// projection.y() < 0 && 
            dist >= config_.min_dist && 
            dist <= config_.max_dist && 
            angle < config_.angle_threshold &&
            right_dist > 0.0f) { // CHANGE THIS FOR LEFT WALL
            right_candidates.push_back(line);
        }
        if (// projection.x() > 0 && 
            dist >= config_.min_dist && 
            dist <= config_.max_dist && 
            angle > config_.angle_threshold &&
            forward_dist > 0.0f) {
            far_candidates.push_back(line);
        }
        //spdlog::info("Drone pos: ({:.2f}, {:.2f}), heading: {:.2f}",
        //     drone_pos.x(), drone_pos.y(), drone_heading);

        //spdlog::info("Projection: ({:.2f}, {:.2f}), dist: {:.2f}, angle: {:.2f}",
        //     projection.x(), projection.y(), dist, angle);
    }
    

    for (const auto& r : right_candidates) {
        for (const auto& f : far_candidates) {
            Eigen::Vector2f intersection_coordinates{};
            bool intersect = line_intersection(r.asEigen(), f.asEigen(), intersection_coordinates);
            if (intersect){
                float wall_angle = angle_between_lines(r.asEigen(), f.asEigen());
                if (wall_angle >= config_.min_angle && wall_angle <= config_.max_angle) {
                    potential_corners.push_back({r, f, intersection_coordinates});
                }
            }
        }
    } 
    return potential_corners;   
} 

Eigen::Vector2f PoolExplorationPlanner::project_drone_to_line( 
    const Eigen::Vector2f& drone_pos,
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line) {
    const Eigen::Vector2f& p0 = line.first;
    const Eigen::Vector2f& p1 = line.second;

    Eigen::Vector2f dir_vec = p1 - p0;
    float len_squared = dir_vec.squaredNorm();

    if (len_squared < 1e-6f) {// segmentet er egentlig bare et punkt, ikke en linje,
        return p0;  // Annen unntakshåndtering?
    }

    float t = (drone_pos - p0).dot(dir_vec) / len_squared;
    t = std::clamp(t, 0.0f, 1.0f);   // projeksjon må ligge på segmentet

    return p0 + t * dir_vec;
}

float PoolExplorationPlanner::line_heading_angle_difference( 
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, 
    float drone_heading) {
    float line_angle = atan2(line.second.y() - line.first.y(), line.second.x() - line.first.x());
    float diff = std::fmod(line_angle - drone_heading, 2*M_PI);
    
    if (diff < -M_PI) diff += 2*M_PI;
    if (diff >  M_PI) diff -= 2*M_PI;

    return std::abs(diff);
}

 bool PoolExplorationPlanner::line_intersection( 
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

float PoolExplorationPlanner::angle_between_lines(
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line0,
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line1) {
    Eigen::Vector2f v0 = (line0.second - line0.first).normalized();
    Eigen::Vector2f v1 = (line1.second - line1.first).normalized();
    
    float dot = std::clamp(v0.dot(v1), -1.0f, 1.0f); //clamper fordi..

    return std::acos(dot);     
}

Eigen::Vector2f PoolExplorationPlanner::compute_inward_normal(
    const LineSegment& line,
    const Eigen::Vector2f& drone_pos)
{
    Eigen::Vector2f p0 = line.asEigen().first;
    Eigen::Vector2f p1 = line.asEigen().second;

    Eigen::Vector2f dir = (p1 - p0).normalized();
    Eigen::Vector2f n(-dir.y(), dir.x());

    Eigen::Vector2f midpoint = 0.5f * (p0 + p1);

    if ((drone_pos - midpoint).dot(n) < 0.0f) {
        n = -n;
    }

    return n.normalized();
}

// NEDENFOR ER GRID LOGIKKEN, BRUKES IKKE ENDA <3
# if 0
// URELEVANT NÅ, LAG NY KONSTRUKTØR TO DO
// Lage egen config til griddet??
PoolExplorationPlanner::PoolExplorationPlanner(
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
//URELEVANT NÅ
void PoolExplorationPlanner::initialize_grid() {
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
// URELEVANT NÅ
const nav_msgs::msg::OccupancyGrid& PoolExplorationPlanner::grid() const {
    return grid_;
}
// URELEVANT NÅ
void PoolExplorationPlanner::insert_line_in_grid(
    const std::vector<LineSegment>& segments) {
    for (const LineSegment& seg : segments) {

        int x0 = static_cast<int>((seg.p0.x - grid_.info.origin.position.x) / grid_.info.resolution);
        int y0 = static_cast<int>((seg.p0.y - grid_.info.origin.position.y) / grid_.info.resolution);
        int x1 = static_cast<int>((seg.p1.x - grid_.info.origin.position.x) / grid_.info.resolution);
        int y1 = static_cast<int>((seg.p1.y - grid_.info.origin.position.y) / grid_.info.resolution);

        bresenham_line_algoritm(x0, y0, x1, y1);
    }
}
// URELEVANT NÅ
void PoolExplorationPlanner::bresenham_line_algoritm(
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
// URELEVANT NÅ
void PoolExplorationPlanner::set_grid_cell(
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

#endif
}  // namespace vortex::pool_exploration

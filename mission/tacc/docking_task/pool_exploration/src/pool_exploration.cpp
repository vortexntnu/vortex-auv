#include <cstdlib>
#include <pool_exploration/pool_exploration.hpp>
#include <spdlog/spdlog.h> // til testing

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
    std::vector<LineSegment> left_candidates;
    std::vector<LineSegment> far_candidates;
    std::vector<CandidateCorner> potential_corners;

    // Finner ut hva som er foran ogsånn? (?)
    Eigen::Vector2f forward(std::cos(drone_heading), std::sin(drone_heading));
    Eigen::Vector2f right(-std::sin(drone_heading), std::cos(drone_heading));

    for (const auto& line : lines) {

        spdlog::info("Line: ({:.2f}, {:.2f}) -> ({:.2f}, {:.2f})",
            line.asEigen().first.x(), line.asEigen().first.y(),
            line.asEigen().second.x(), line.asEigen().second.y());

        Eigen::Vector2f projection = project_drone_to_line(drone_pos, line.asEigen());
        Eigen::Vector2f rel = projection - drone_pos;

        float dist = rel.norm(); 
        float angle = line_heading_angle_difference(line.asEigen(), drone_heading);

        float forward_dist = rel.dot(forward);
        float right_dist = rel.dot(right);

        spdlog::info("  Projection: ({:.2f}, {:.2f})",
            projection.x(), projection.y());

        spdlog::info("  Rel: ({:.2f}, {:.2f}), dist: {:.2f}",
            rel.x(), rel.y(), dist);

        spdlog::info("  Angle: {:.2f} rad ({:.1f} deg)",
            angle, angle * 180.0 / M_PI);

        spdlog::info("  forward_dist: {:.2f}, right_dist: {:.2f}",
            forward_dist, right_dist);

        spdlog::info("Angle: {:.2f}, right_dist: {:.2f}, forward_dist: {:.2f}",
             angle, right_dist, forward_dist);

        if (dist >= config_.min_dist && 
            dist <= config_.max_dist && 
            angle < config_.angle_threshold  &&
            right_dist > config_.right_dist
            ) { 
            spdlog::info("  → Classified as RIGHT candidate");
            right_candidates.push_back(line);
        }
        else if (dist >= config_.min_dist && 
            dist <= config_.max_dist && 
            angle > config_.angle_threshold  &&
            forward_dist > 0.0f
            ) {
            spdlog::info("  → Classified as FAR candidate");
            far_candidates.push_back(line);
        }
        
        else if (dist >= config_.min_dist && 
            dist <= config_.max_dist && 
            angle < config_.angle_threshold  &&
            right_dist < 0.0f
            ) { 
                spdlog::info("  → Classified as LEFT candidate");
            left_candidates.push_back(line);
        }
        else {
                spdlog::info("  → REJECTED by angle");
        }
    }
    spdlog::info("==== SUMMARY ====");
    spdlog::info("Right candidates: {}", right_candidates.size());
    spdlog::info("Far candidates: {}", far_candidates.size());

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

    return p0 + t * dir_vec;
}

float PoolExplorationPlanner::line_heading_angle_difference( 
    const std::pair<Eigen::Vector2f, Eigen::Vector2f>& line, 
    float drone_heading) {
    Eigen::Vector2f dir = line.second - line.first;
    float norm = dir.norm();

    if (norm < 1e-6f)
        return std::numeric_limits<float>::infinity();

    dir /= norm;

    Eigen::Vector2f heading_vec(std::cos(drone_heading), std::sin(drone_heading));
    float dot = std::abs(dir.dot(heading_vec));
    return std::acos(dot);
/*
    float line_angle = atan2(line.second.y() - line.first.y(), line.second.x() - line.first.x());
    float diff = std::fmod(line_angle - drone_heading, 2*M_PI);
    
    if (diff < -M_PI) diff += 2*M_PI;
    if (diff >  M_PI) diff -= 2*M_PI;

    return std::abs(diff);
    */
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

    Eigen::Vector2f v0 = line0.second - line0.first;
    Eigen::Vector2f v1 = line1.second - line1.first;

    float n0 = v0.norm();
    float n1 = v1.norm();
    
    if (n0 < 1e-6f || n1 < 1e-6f)
        return std::numeric_limits<float>::infinity();

    v0 /= n0;
    v1 /= n1;

    float dot = std::abs(v0.dot(v1));
    dot = std::clamp(dot, 0.0f, 1.0f);

    return std::acos(dot);     
}

Eigen::Vector2f PoolExplorationPlanner::compute_inward_normal(
    const LineSegment& line,
    const Eigen::Vector2f& drone_pos)
{
    Eigen::Vector2f p0 = line.asEigen().first;
    Eigen::Vector2f p1 = line.asEigen().second;

    // Normaliser linjeretningen slik at retningen er entydig uavhengig av p0 og p1 rekkefølge
    if ((p1 - p0).x() < 0 || ((p1 - p0).x() == 0 && (p1 - p0).y() < 0)) {
        // Bytt om slik at vektoren alltid peker "mot høyre" eller oppover
        std::swap(p0, p1);
    }

    Eigen::Vector2f dir = (p1 - p0).normalized();
    Eigen::Vector2f n(-dir.y(), dir.x());

    Eigen::Vector2f midpoint = 0.5f * (p0 + p1);

    if ((drone_pos - midpoint).dot(n) < 0.0f) {
        n = -n;
    }

    return n.normalized();
}

}  // namespace vortex::pool_exploration

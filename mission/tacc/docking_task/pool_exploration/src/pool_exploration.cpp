#include <cstdlib>
#include <pool_exploration/pool_exploration.hpp>
#include <spdlog/spdlog.h> // til testing

#include <vector>
#include <vortex/utils/types.hpp>
#include <Eigen/src/Core/Matrix.h>

namespace vortex::pool_exploration{

PoolExplorationPlanner::PoolExplorationPlanner(const PoolExplorationPlannerConfig& config)
    : config_(config) {}
// DELARERE
inline Eigen::Vector2f to_eigen(const vortex::utils::types::Point2D p) {
    return {p.x, p.y};
}

Eigen::Vector2f PoolExplorationPlanner::estimate_docking_position(
    const CornerEstimate& estimated_corner,
    const Eigen::Vector2f& drone_pos) {
    Eigen::Vector2f right_normal = compute_normal_towards_point(estimated_corner.right_wall, drone_pos);
    Eigen::Vector2f far_normal   = compute_normal_towards_point(estimated_corner.far_wall, drone_pos);

    Eigen::Vector2f docking_estimate = estimated_corner.corner_point
        + right_normal * config_.right_wall_offset_m
        + far_normal   * config_.far_wall_offset_m;

    return docking_estimate;
}

CornerEstimate PoolExplorationPlanner::select_best_corner(
    const std::vector<CornerEstimate>& possible_corners,
    const Eigen::Vector2f& drone_pos) {
    if (possible_corners.empty()) {
        throw std::runtime_error("No candidate corners available");  //finne bedre unnntakshåndtering?
    }

    float min_distance = std::numeric_limits<float>::max();
    CornerEstimate best_corner = possible_corners.front();

    for (const auto& corner : possible_corners) {
        float distance = (corner.corner_point - drone_pos).squaredNorm();

        if (distance < min_distance) {
            min_distance = distance;
            best_corner = corner;
        }
    }
    return best_corner;
}

std::vector<CornerEstimate> PoolExplorationPlanner::find_corner_estimates(
    const std::vector<vortex::utils::types::LineSegment2D>& lines,
    const Eigen::Vector2f& drone_pos, 
    float drone_heading) {

    std::vector<vortex::utils::types::LineSegment2D> right_candidates;
    std::vector<vortex::utils::types::LineSegment2D> left_candidates;
    std::vector<vortex::utils::types::LineSegment2D> far_candidates;
    std::vector<CornerEstimate> potential_corners;

    // Finner ut hva som er foran ogsånn? (?)
    Eigen::Vector2f forward(std::cos(drone_heading), std::sin(drone_heading));
    Eigen::Vector2f right(-std::sin(drone_heading), std::cos(drone_heading));

    for (const auto& line : lines) {
        // const auto eigen_line = line.asEigen();
        const Eigen::Vector2f p0 = to_eigen(line.p0);
        const Eigen::Vector2f p1 = to_eigen(line.p1);
        
        spdlog::info("Line: ({:.2f}, {:.2f}) -> ({:.2f}, {:.2f})",
            p0.x(), p0.y(), p1.x(), p1.y());

        Eigen::Vector2f projection = project_point_onto_line(drone_pos, line);
        Eigen::Vector2f rel = projection - drone_pos;

        float dist = rel.norm(); 
        float angle = angle_between_line_and_heading(line, drone_heading);

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

        if (dist >= config_.min_wall_distance_m && 
            dist <= config_.max_wall_distance_m && 
            angle < config_.far_wall_heading_angle_threshold  &&
            right_dist > config_.right_dist
            ) { 
            spdlog::info("  → Classified as RIGHT candidate");
            right_candidates.push_back(line);
        }
        else if (dist >= config_.min_wall_distance_m && 
            dist <= config_.max_wall_distance_m && 
            angle > config_.far_wall_heading_angle_threshold  &&
            forward_dist > 0.0f
            ) {
            spdlog::info("  → Classified as FAR candidate");
            far_candidates.push_back(line);
        }
        
        else if (dist >= config_.min_wall_distance_m && 
            dist <= config_.max_wall_distance_m && 
            angle < config_.far_wall_heading_angle_threshold  &&
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

    for (const auto& right_wall : right_candidates) {
        for (const auto& far_wall : far_candidates) {
            Eigen::Vector2f intersection_coordinates{};
            bool intersect = compute_line_intersection(right_wall, far_wall, intersection_coordinates);
            if (intersect){
                float wall_angle = angle_between_lines(right_wall, far_wall);
                if (wall_angle >= config_.min_corner_angle_rad && wall_angle <= config_.max_corner_angle_rad) {
                    potential_corners.push_back({right_wall, far_wall, intersection_coordinates});
                }
            }
        }
    } 
    return potential_corners;   
} 

Eigen::Vector2f PoolExplorationPlanner::project_point_onto_line( 
    const Eigen::Vector2f& drone_pos,
    const vortex::utils::types::LineSegment2D& line) {
    const Eigen::Vector2f& p0 = to_eigen(line.p0);
    const Eigen::Vector2f& p1 = to_eigen(line.p1);

    Eigen::Vector2f dir_vec = p1 - p0;
    float len_squared = dir_vec.squaredNorm();

    if (len_squared < 1e-6f) {// segmentet er egentlig bare et punkt, ikke en linje,
        return p0;  // Annen unntakshåndtering?
    }

    float t = (drone_pos - p0).dot(dir_vec) / len_squared;

    return p0 + t * dir_vec;
}

float PoolExplorationPlanner::angle_between_line_and_heading( 
    const vortex::utils::types::LineSegment2D& line, 
    float drone_heading) {
    Eigen::Vector2f dir = to_eigen(line.p1) - to_eigen(line.p0);
    const float norm = dir.norm();

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

bool PoolExplorationPlanner::compute_line_intersection( 
    const vortex::utils::types::LineSegment2D& line0,
    const vortex::utils::types::LineSegment2D& line1,
    Eigen::Vector2f& intersection) {
    const Eigen::Vector2f p00 = to_eigen(line0.p0);
    const Eigen::Vector2f p01 = to_eigen(line0.p1);
    const Eigen::Vector2f p10 = to_eigen(line1.p0);
    const Eigen::Vector2f p11 = to_eigen(line1.p1);

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
    const vortex::utils::types::LineSegment2D& line0,
    const vortex::utils::types::LineSegment2D&  line1) {

    Eigen::Vector2f v0 = to_eigen(line0.p1) - to_eigen(line0.p0);
    Eigen::Vector2f v1 = to_eigen(line1.p1) - to_eigen(line1.p0);

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

Eigen::Vector2f PoolExplorationPlanner::compute_normal_towards_point(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos)
{
    Eigen::Vector2f p0 = to_eigen(line.p0);
    Eigen::Vector2f p1 = to_eigen(line.p1);

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

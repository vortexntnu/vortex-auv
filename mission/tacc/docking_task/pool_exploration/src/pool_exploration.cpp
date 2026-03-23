#include <cstdlib>
#include <pool_exploration/pool_exploration.hpp>
#include <spdlog/spdlog.h> // til testing

#include <vector>
#include <vortex/utils/types.hpp>
#include <Eigen/src/Core/Matrix.h>

namespace vortex::pool_exploration{

PoolExplorationPlanner::PoolExplorationPlanner(const PoolExplorationPlannerConfig& config)
    : config_(config) {}

# if 1
std::vector<CornerEstimate> PoolExplorationPlanner::find_corner_estimates(
    const std::vector<vortex::utils::types::LineSegment2D>& lines,
    const Eigen::Vector2f& drone_pos,
    float drone_heading) const {
    std::vector<vortex::utils::types::LineSegment2D> right_wall_candidates;
    std::vector<vortex::utils::types::LineSegment2D> far_wall_candidates;
    std::vector<CornerEstimate> corner_estimates;

    for (const auto& line : lines) {
        const WallClassification classification = classify_wall(line, drone_pos, drone_heading);

        if (classification == WallClassification::RightWall) {
            right_wall_candidates.push_back(line);
        } 
        else if (classification == WallClassification::FarWall) {
            far_wall_candidates.push_back(line);
        }
    }

    spdlog::info("==== SUMMARY ====");
    spdlog::info("Right wall candidates: {}", right_wall_candidates.size());
    spdlog::info("Far wall candidates: {}", far_wall_candidates.size());

    for (const auto& right_wall : right_wall_candidates) {
        for (const auto& far_wall : far_wall_candidates) {
            Eigen::Vector2f corner_intersection;
            if (!compute_line_intersection(right_wall, far_wall, corner_intersection)) {
                continue;
            }

            const float wall_angle = angle_between_lines(right_wall, far_wall);

            if (wall_angle >= config_.min_corner_angle_rad &&
                wall_angle <= config_.max_corner_angle_rad) {
                corner_estimates.push_back({right_wall, far_wall, corner_intersection});
            }
        }
    }

    return corner_estimates;
}
#endif

CornerEstimate PoolExplorationPlanner::select_best_corner(
    const std::vector<CornerEstimate>& possible_corners,
    const Eigen::Vector2f& drone_pos) const {
    // if (possible_corners.empty()) {throw std::runtime_error("No candidate corners available"); } // UNNTAKSHÅNDTERING I ROS I STEDET

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

Eigen::Vector2f PoolExplorationPlanner::estimate_docking_position(
    const CornerEstimate& estimated_corner,
    const Eigen::Vector2f& drone_pos) const {
    Eigen::Vector2f right_normal = compute_normal_towards_point(estimated_corner.right_wall, drone_pos);
    Eigen::Vector2f far_normal   = compute_normal_towards_point(estimated_corner.far_wall, drone_pos);

    Eigen::Vector2f docking_estimate = estimated_corner.corner_point
        + right_normal * config_.right_wall_offset_m
        + far_normal   * config_.far_wall_offset_m;

    return docking_estimate;
}

WallClassification PoolExplorationPlanner::classify_wall(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos,
    float drone_heading) const {
    const Eigen::Vector2f p0 = to_eigen(line.p0);
    const Eigen::Vector2f p1 = to_eigen(line.p1);

    const Eigen::Vector2f dir = p1 - p0;
    if (dir.squaredNorm() < 1e-6f) { // ENDRE KRAV TIL LENGDE PÅ LINJE?
        spdlog::info("  -> REJECTED: degenerate line");
        return WallClassification::Rejected;
    }
    // TIL LOGGING
        spdlog::info("Line: ({:.2f}, {:.2f}) -> ({:.2f}, {:.2f})",
                    p0.x(), p0.y(), p1.x(), p1.y());
    //
    const Eigen::Vector2f projection = project_drone_onto_line(drone_pos, line);
    const Eigen::Vector2f rel = projection - drone_pos;

    const float distance = rel.norm();
    // TIL LOGGING
        spdlog::info("  Projection: ({:.2f}, {:.2f})", projection.x(), projection.y());
        spdlog::info("  Rel: ({:.2f}, {:.2f}), dist: {:.2f}",
                    rel.x(), rel.y(), distance);
    //
    if (distance < config_.min_wall_distance_m ||
        distance > config_.max_wall_distance_m) {
        spdlog::info("  -> REJECTED by distance");
        return WallClassification::Rejected;
    }

    const float heading_wall_angle = angle_between_line_and_heading(line, drone_heading);
    spdlog::info("  Angle: {:.2f} rad ({:.1f} deg)",
             heading_wall_angle, heading_wall_angle * 180.0 / M_PI);

    // RIGHT WALL: projection has negative y-value in NED, wall is approximately parallel to heading
    if (projection.y() < config_.right_wall_max_y_m &&
        heading_wall_angle < config_.parallel_heading_angle_threshold_rad) {
        spdlog::info("  -> Classified as RIGHT candidate");
        return WallClassification::RightWall;
    }

    // FAR WALL: projection has positive x-value, wall is approximately perpendicular to heading
    if (projection.x() > config_.far_wall_min_x_m &&
        heading_wall_angle > config_.perpendicular_heading_angle_threshold_rad) {
        spdlog::info("  -> Classified as FAR candidate");
        return WallClassification::FarWall;
    }
    spdlog::info("  -> REJECTED by geometry");
    return WallClassification::Rejected;
}

Eigen::Vector2f PoolExplorationPlanner::project_drone_onto_line( 
    const Eigen::Vector2f& drone_pos,
    const vortex::utils::types::LineSegment2D& line) const {
    const Eigen::Vector2f p0 = to_eigen(line.p0);
    const Eigen::Vector2f p1 = to_eigen(line.p1);

    const Eigen::Vector2f line_direction = p1 - p0;
    const float line_length_squared = line_direction.squaredNorm();

    const float projection_parameter = (drone_pos - p0).dot(line_direction) / line_length_squared;
    const Eigen::Vector2f projection_point = p0 + projection_parameter * line_direction;

    return projection_point;
}

float PoolExplorationPlanner::angle_between_line_and_heading( 
    const vortex::utils::types::LineSegment2D& line, 
    float drone_heading) const {
    Eigen::Vector2f wall_direction = to_eigen(line.p1) - to_eigen(line.p0);
    const float wall_length = wall_direction.norm();

    if (wall_length < 1e-6f) {
        return std::numeric_limits<float>::infinity();
    }

    wall_direction /= wall_length;
    Eigen::Vector2f heading_direction(
        std::cos(drone_heading), 
        std::sin(drone_heading));

    float cos_heading_wall_angle =
        std::abs(wall_direction.dot(heading_direction)); //rekkefølge p0 og p1 urelevant
    cos_heading_wall_angle = std::clamp(cos_heading_wall_angle, 0.0f, 1.0f); // unngå små avvik

    return std::acos(cos_heading_wall_angle);
}

bool PoolExplorationPlanner::compute_line_intersection( 
    const vortex::utils::types::LineSegment2D& line0,
    const vortex::utils::types::LineSegment2D& line1,
    Eigen::Vector2f& intersection) const {
    const Eigen::Vector2f p00 = to_eigen(line0.p0);
    const Eigen::Vector2f p01 = to_eigen(line0.p1);
    const Eigen::Vector2f p10 = to_eigen(line1.p0);
    const Eigen::Vector2f p11 = to_eigen(line1.p1);

    float determinant = (p00.x()-p01.x())*(p10.y()-p11.y()) - (p00.y()-p01.y())*(p10.x()-p11.x());

    if (std::abs(determinant) < 1e-6f) 
        return false; // parallelle linjer

    intersection.x() = ((p00.x()*p01.y()-p00.y()*p01.x())*(p10.x()-p11.x()) -
                        (p00.x()-p01.x())*(p10.x()*p11.y()-p10.y()*p11.x()))/determinant;
    intersection.y() = ((p00.x()*p01.y()-p00.y()*p01.x())*(p10.y()-p11.y()) -
                        (p00.y()-p01.y())*(p10.x()*p11.y()-p10.y()*p11.x()))/determinant;
    return true;
}

float PoolExplorationPlanner::angle_between_lines(
    const vortex::utils::types::LineSegment2D& line0,
    const vortex::utils::types::LineSegment2D&  line1) const {

    Eigen::Vector2f line0_direction =
        to_eigen(line0.p1) - to_eigen(line0.p0);

    Eigen::Vector2f line1_direction =
        to_eigen(line1.p1) - to_eigen(line1.p0);

    const float line0_length = line0_direction.norm();
    const float line1_length = line1_direction.norm();

    if (line0_length < 1e-6f || line1_length < 1e-6f) {
        return std::numeric_limits<float>::infinity();
    }

    line0_direction /= line0_length;
    line1_direction /= line1_length;

    float cos_angle_between_lines =
        std::abs(line0_direction.dot(line1_direction));

    cos_angle_between_lines =
        std::clamp(cos_angle_between_lines, 0.0f, 1.0f);

    return std::acos(cos_angle_between_lines);
}

Eigen::Vector2f PoolExplorationPlanner::compute_normal_towards_point(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos) const
{
    Eigen::Vector2f p0 = to_eigen(line.p0);
    Eigen::Vector2f p1 = to_eigen(line.p1);

    Eigen::Vector2f line_direction = p1 - p0;

    if (line_direction.x() < 0.0f ||
        (line_direction.x() == 0.0f && line_direction.y() < 0.0f)) {
        std::swap(p0, p1);
        line_direction = p1 - p0;
    }

    const float line_length = line_direction.norm();
    if (line_length < 1e-6f) {
        return Eigen::Vector2f::Zero();
    }

    line_direction /= line_length;

    Eigen::Vector2f normal(-line_direction.y(), line_direction.x());

    const Eigen::Vector2f line_midpoint = 0.5f * (p0 + p1);
    const Eigen::Vector2f midpoint_to_drone = drone_pos - line_midpoint;

    if (midpoint_to_drone.dot(normal) < 0.0f) {
        normal = -normal;
    }

    return normal;
}

// klassifisere hjørne utfra relativ avstand til drone
# if 0
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
    Eigen::Vector2f right(std::sin(drone_heading), -std::cos(drone_heading)); // RIKTIG NED?

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
#endif

}  // namespace vortex::pool_exploration

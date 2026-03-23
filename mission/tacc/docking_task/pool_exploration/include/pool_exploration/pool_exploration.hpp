#ifndef POOL_EXPLORATION_HPP
#define POOL_EXPLORATION_HPP

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vortex/utils/types.hpp>

namespace vortex::pool_exploration{

/**
 * @brief Represents a candidate docking corner formed by a right wall and a far wall.
 *
 * A corner estimate consists of two detected wall segments and their intersection
 * point in the planning frame.
 */
struct CornerEstimate {
    /** @brief Line segment classified as the right wall. */
    vortex::utils::types::LineSegment2D right_wall;

    /** @brief Line segment classified as the far wall. */
    vortex::utils::types::LineSegment2D far_wall;

    /** @brief Intersection point between the two walls. */
    Eigen::Vector2f corner_point;
};

/**
 * @brief Classification result for a detected wall segment.
 */
enum class WallClassification {
    /** @brief Wall segment classified as the right wall. */
    RightWall,

    /** @brief Wall segment classified as the left wall. */
    LeftWall,

    /** @brief Wall segment classified as the far wall. */
    FarWall,

    /** @brief Wall segment does not satisfy any classification rule. */
    Rejected
};

/**
 * @brief Convert a Point2D to an Eigen 2D vector.
 *
 * @param point Input point.
 * @return Equivalent Eigen::Vector2f.
 */
inline Eigen::Vector2f to_eigen(const vortex::utils::types::Point2D point) {
    return {point.x, point.y};
}

/**
 * @brief Configuration parameters for the pool exploration planner.
 *
 * These parameters control wall classification, corner validation, and docking
 * point estimation.
 */
struct PoolExplorationPlannerConfig {
    /** @brief Minimum allowed distance from the drone to a candidate wall projection [m]. */
    float min_wall_distance_m; 

    /** @brief Maximum allowed distance from the drone to a candidate wall projection [m]. */
    float max_wall_distance_m;

    // REMOVE THIS IF OTHER CODE WORKS, TO DO
    float far_wall_heading_angle_threshold; 

    /** @brief Maximum angle between heading and a wall for it to be considered parallel [rad]. */
    float parallel_heading_angle_threshold_rad;

    /** @brief Minimum angle between heading and a wall for it to be considered perpendicular [rad]. */
    float perpendicular_heading_angle_threshold_rad;

    /** @brief Minimum allowed angle between right wall and far wall for a valid corner [rad]. */
    float min_corner_angle_rad;

    /** @brief Maximum allowed angle between right wall and far wall for a valid corner [rad]. */
    float max_corner_angle_rad;

    // REMOVE THIS IF OTHER CODE WORKS, TO DO
    float right_dist; 

    /** @brief Maximum y-value used when checking whether a projection belongs to the right wall. */
    float right_wall_max_y_m;

    /** @brief Minimum x-value used when checking whether a projection belongs to the far wall. */
    float far_wall_min_x_m;

    /** @brief Offset from the right wall when estimating docking position [m]. */
    float right_wall_offset_m;

    /** @brief Offset from the far wall when estimating docking position [m]. */
    float far_wall_offset_m;

    /** @brief 
    * Selects whether to prefer the right-side corner in the mission logic. 
    */
    // HAS NOT BEEN IMPLEMENTED YET, TO DO
    int choose_right_corner;
};

/**
 * @brief Planner for detecting pool corners and estimating a docking position.
 *
 * The planner classifies detected line segments as candidate pool walls, combines
 * compatible wall pairs into corner estimates, and computes a docking point from
 * the selected corner.
 */
class PoolExplorationPlanner {
public:
    /**
     * @brief Construct a new planner instance.
     *
     * @param config Planner configuration parameters.
     */
    PoolExplorationPlanner(
        const PoolExplorationPlannerConfig& config);

    /**
     * @brief Estimate a docking position from a validated corner estimate.
     *
     * The docking point is computed by offsetting the detected corner along the
     * inward-facing normals of the right wall and far wall.
     *
     * @param estimated_corner Corner estimate containing the two wall segments and their intersection.
     * @param drone_pos Current drone position in the same frame as the corner estimate.
     * @return Estimated docking position in the planning frame.
     */
    Eigen::Vector2f estimate_docking_position(
        const CornerEstimate& estimated_corner,
        const Eigen::Vector2f& drone_pos);

    /**
     * @brief Find all valid corner estimates from a set of detected line segments.
     *
     * Each line segment is first classified as a candidate wall. Candidate right walls
     * and far walls are then paired, and their intersection is checked. A pair is
     * accepted as a corner if the angle between the two walls lies within the configured
     * corner-angle interval.
     *
     * @param lines Input line segments expressed in the planning frame.
     * @param drone_pos Current drone position in the same frame as the lines.
     * @param drone_heading Current drone heading [rad].
     * @return Vector of valid corner estimates.
     */
    //  EVT LAGE FOR VENSTRE VEGG, TO DO
    std::vector<CornerEstimate> find_corner_estimates( 
        const std::vector<vortex::utils::types::LineSegment2D>& lines, 
        const Eigen::Vector2f& drone_pos, // sjekk at får inn pos og heading riktig?
        float drone_heading);

    /**
     * @brief Select the best corner estimate among all candidates.
     *
     * The current implementation selects the corner whose intersection point is
     * closest to the drone.
     *
     * @param possible_corners All valid corner candidates.
     * @param drone_pos Current drone position in the planning frame.
     * @return Best corner estimate.
     *
     * @throws std::runtime_error if the input vector is empty. 
     // ENDRE UNNTAKSHÅNDTERING?, TO DO
     */
    CornerEstimate select_best_corner( 
        const std::vector<CornerEstimate>& possible_corners,
        const Eigen::Vector2f& drone_pos);
    
private:
   
    /**
     * @brief Project a point orthogonally onto the infinite line defined by a line segment.
     *
     * If the segment is degenerate, the first endpoint is returned. ANNEN HÅNDTERING? TO DO
     *
     * @param drone_pos Point to be projected.
     * @param line Line segment defining the projection line.
     * @return Projected point in the same frame.
     */
    // PROJEKSJONSFORMELEN, TO DO
    
    Eigen::Vector2f project_drone_onto_line(
        const Eigen::Vector2f& drone_pos, // punktet som projiseres
        const vortex::utils::types::LineSegment2D& line);

    /**
    * @brief Compute the smallest unsigned angle between a wall segment and the drone heading.
    *
    * The wall direction is compared to the drone heading direction using the absolute
    * dot product, making the result independent of the line endpoint ordering.
    *
    * The returned angle lies in [0, π/2]:
    *  - ≈ 0 rad → wall is parallel to heading
    *  - ≈ π/2 rad → wall is perpendicular to heading
    *
    * If the segment is degenerate (near-zero length), +∞ is returned.
    * The dot product is clamped to [0, 1] to ensure numerical stability in acos().
    *
    * @param line Wall segment.
    * @param drone_heading Drone heading [rad].
    * @return Smallest unsigned angle between wall and heading [rad], or +∞ if degenerate.
    */
    float angle_between_line_and_heading(
        const vortex::utils::types::LineSegment2D& line, 
        float drone_heading);

    /**
     * @brief Compute the intersection point of two infinite lines.
     *
     * The lines are defined by the endpoints of the two input line segments.
     * The method returns false if the lines are parallel.
     *
     * @param line0 First line segment.
     * @param line1 Second line segment.
     * @param intersection_coordinates Output intersection point.
     * @return true if the lines intersect.
     * @return false if the lines are parallel.
     */
    bool compute_line_intersection( 
        const vortex::utils::types::LineSegment2D& line0,
        const vortex::utils::types::LineSegment2D& line1,
        Eigen::Vector2f& intersection_coordinates);
    
    /**
     * @brief Compute the smallest angle between two line segments.
     *
     * The result is the unsigned smallest angle between the direction vectors of
     * the two segments.
     *
     * @param line0 First line segment.
     * @param line1 Second line segment.
     * @return Angle between the two lines [rad].
     */
    float angle_between_lines(
        const vortex::utils::types::LineSegment2D& line0,
        const vortex::utils::types::LineSegment2D& line1);
    /**
     * @brief Compute the unit normal of a line pointing towards a reference point.
     *
     * This is used when computing a docking position offset from a detected wall.
     *
     * @param line Input line segment.
     * @param drone_pos Reference point used to choose normal direction.
     * @return Unit normal pointing towards the reference point.
     */
    Eigen::Vector2f compute_normal_towards_point(const vortex::utils::types::LineSegment2D& line, const Eigen::Vector2f& drone_pos);

    /**
    * @brief Classify a line segment as a right wall, far wall, or rejected.
    *
    * A line is classified using:
    * - the distance from the drone to the orthogonal projection on the line,
    * - the position of that projection relative to the expected mission geometry,
    * - and the unsigned angle between the wall direction and drone heading.
    *
    * Right-wall candidates must satisfy the configured right-side projection test
    * and be approximately parallel to the drone heading.
    *
    * Far-wall candidates must satisfy the configured forward projection test
    * and be approximately perpendicular to the drone heading.
    *
     * @param line Input line segment.
     * @param drone_pos Current drone position.
     * @param drone_heading Current drone heading [rad].
     * @return Wall classification result.
     */
    WallClassification classify_wall(
    const vortex::utils::types::LineSegment2D& line,
    const Eigen::Vector2f& drone_pos,
    float drone_heading);

    /** @brief Planner configuration. */
    PoolExplorationPlannerConfig config_;
};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_HPP

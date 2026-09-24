#ifndef LANDMARK_TARGETS__GEOMETRY_HPP_
#define LANDMARK_TARGETS__GEOMETRY_HPP_

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>

namespace vortex::mission {

enum class Side : uint8_t { LEFT, RIGHT };

/**
 * @brief Distance from the vehicle to @p point along the vehicle's forward
 * axis (x in the body frame). > 0 = in front of the vehicle.
 */
double forward_distance(const vortex::utils::types::Pose& vehicle,
                        const Eigen::Vector3d& point);

/**
 * @brief Which side of the vehicle @p point is on, in the horizontal plane
 * (cross product with the vehicle heading; NED, z down).
 * A point straight ahead is reported as LEFT.
 */
Side side_of(const vortex::utils::types::Pose& vehicle,
             const Eigen::Vector3d& point);

/**
 * @brief Heading perpendicular to the line a -> b (odom x, y), choosing the
 * one of the two perpendiculars that is closest to @p current_yaw.
 */
double perpendicular_heading(const Eigen::Vector2d& a,
                             const Eigen::Vector2d& b,
                             double current_yaw);

enum class CourseState : uint8_t { UNSET, COARSE, GATE_LOCKED };

/**
 * @brief The course frame: origin at the gate (or start estimate), x through
 * the gate, y to the right, z down (NED). The same axes as TF
 * nautilus/course, so a tf2 transform into that frame gives the same result as
 * to_course().
 */
struct CourseFrame {
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    double through_yaw{0.0};  // course direction in odom [rad]
    CourseState state{CourseState::UNSET};
};

Eigen::Vector2d to_course(const CourseFrame& course,
                          const Eigen::Vector2d& odom_xy);
Eigen::Vector2d from_course(const CourseFrame& course,
                            const Eigen::Vector2d& course_xy);

}  // namespace vortex::mission

#endif  // LANDMARK_TARGETS__GEOMETRY_HPP_

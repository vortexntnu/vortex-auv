#include "landmark_targets/geometry.hpp"
#include <cmath>
#include <vortex/utils/math.hpp>

namespace vortex::mission {

namespace {

double yaw_of(const vortex::utils::types::Pose& pose) {
    const Eigen::Quaterniond q = pose.ori_quaternion().normalized();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace

double forward_distance(const vortex::utils::types::Pose& vehicle,
                        const Eigen::Vector3d& point) {
    const Eigen::Vector3d forward =
        vehicle.ori_quaternion().normalized() * Eigen::Vector3d::UnitX();
    return forward.dot(point - vehicle.pos_vector());
}

Side side_of(const vortex::utils::types::Pose& vehicle,
             const Eigen::Vector3d& point) {
    const double yaw = yaw_of(vehicle);
    const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d d = (point - vehicle.pos_vector()).head<2>();
    // NED (z down): a positive cross product is clockwise, i.e. to the right.
    const double cross = forward.x() * d.y() - forward.y() * d.x();
    return cross > 0.0 ? Side::RIGHT : Side::LEFT;
}

double perpendicular_heading(const Eigen::Vector2d& a,
                             const Eigen::Vector2d& b,
                             double current_yaw) {
    const Eigen::Vector2d d = b - a;
    const double line = std::atan2(d.y(), d.x());
    const double cand1 = vortex::utils::math::ssa(line + M_PI_2);
    const double cand2 = vortex::utils::math::ssa(line - M_PI_2);
    const double e1 = std::abs(vortex::utils::math::ssa(cand1 - current_yaw));
    const double e2 = std::abs(vortex::utils::math::ssa(cand2 - current_yaw));
    return e1 <= e2 ? cand1 : cand2;
}

Eigen::Vector2d to_course(const CourseFrame& course,
                          const Eigen::Vector2d& odom_xy) {
    const double c = std::cos(course.through_yaw);
    const double s = std::sin(course.through_yaw);
    const Eigen::Vector2d d = odom_xy - course.origin;
    // x along the course direction, y to the right, z down: the same axes as
    // TF nautilus/course.
    return {c * d.x() + s * d.y(), -s * d.x() + c * d.y()};
}

Eigen::Vector2d from_course(const CourseFrame& course,
                            const Eigen::Vector2d& course_xy) {
    const double c = std::cos(course.through_yaw);
    const double s = std::sin(course.through_yaw);
    return course.origin +
           Eigen::Vector2d(c * course_xy.x() - s * course_xy.y(),
                           s * course_xy.x() + c * course_xy.y());
}

}  // namespace vortex::mission

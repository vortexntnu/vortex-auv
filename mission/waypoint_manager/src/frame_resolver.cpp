#include "waypoint_manager/frame_resolver.hpp"

namespace vortex::mission {

Pose resolve_pose(const Pose& target, GoalFrame frame, const Pose& start) {
    const Eigen::Quaterniond q0 = start.ori_quaternion().normalized();
    const Eigen::Quaterniond q_target = target.ori_quaternion().normalized();

    switch (frame) {
        case GoalFrame::BODY_RELATIVE:
            return Pose::from_eigen(
                start.pos_vector() + q0 * target.pos_vector(),
                (q0 * q_target).normalized());
        case GoalFrame::WORLD_RELATIVE:
            return Pose::from_eigen(start.pos_vector() + target.pos_vector(),
                                    (q_target * q0).normalized());
        case GoalFrame::WORLD:
        default:
            return target;
    }
}

}  // namespace vortex::mission

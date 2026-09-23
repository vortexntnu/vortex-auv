#include "waypoint_manager/frame_resolver.hpp"

namespace vortex::mission {

Pose resolve_pose(const Pose& target, OffsetFrame frame, const Pose& start) {
    const Eigen::Quaterniond q0 = start.ori_quaternion().normalized();
    const Eigen::Quaterniond q_target = target.ori_quaternion().normalized();

    switch (frame) {
        case OffsetFrame::BODY_RELATIVE:
            return Pose::from_eigen(
                start.pos_vector() + q0 * target.pos_vector(),
                (q0 * q_target).normalized());
        case OffsetFrame::WORLD_RELATIVE:
            return Pose::from_eigen(start.pos_vector() + target.pos_vector(),
                                    (q_target * q0).normalized());
        case OffsetFrame::WORLD:
        default:
            return target;
    }
}

}  // namespace vortex::mission

#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/detail/landmark_array__struct.hpp>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

std::vector<Landmark> LandmarkServerNode::ros_msg_to_landmarks(
    const vortex_msgs::msg::LandmarkArray& msg) const {
    std::vector<Landmark> out;
    out.reserve(msg.landmarks.size());
    for (const auto& lm_msg : msg.landmarks) {
        Landmark lm;
        lm.pose =
            vortex::utils::ros_conversions::ros_pose_to_pose(lm_msg.pose.pose);
        lm.class_key = vortex::filtering::LandmarkClassKey{
            lm_msg.type.value, lm_msg.subtype.value};
        out.push_back(lm);
    }
    return out;
}

vortex_msgs::msg::LandmarkArray LandmarkServerNode::tracks_to_landmark_msgs(
    uint16_t type,
    uint16_t subtype) const {
    vortex_msgs::msg::LandmarkArray out;

    const auto tracks = track_manager_->get_tracks_by_type(
        vortex::filtering::LandmarkClassKey{type, subtype});
    out.landmarks.reserve(tracks.size());

    const auto stamp = this->now();

    out.header.stamp = stamp;
    out.header.frame_id = target_frame_;

    for (const auto* t : tracks) {
        vortex_msgs::msg::Landmark lm;

        lm.type.value = t->class_key.type;
        lm.subtype.value = t->class_key.subtype;
        lm.id = t->id;

        lm.pose.pose =
            vortex::utils::ros_conversions::to_pose_msg(t->to_pose());

        out.landmarks.push_back(std::move(lm));
    }

    return out;
}

}  // namespace vortex::mission

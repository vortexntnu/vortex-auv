#include <vortex/utils/ros/qos_profiles.hpp>
#include "landmark_server/landmark_server_ros.hpp"

namespace vortex::mission {

void LandmarkServerNode::setup_debug_publishers() {
    std::string debug_topic =
        this->declare_parameter<std::string>("debug.topic_name");
    auto qos = vortex::utils::qos_profiles::sensor_data_profile(10);
    landmark_track_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            debug_topic, qos);

    std::string convergence_landmark_topic =
        this->declare_parameter<std::string>(
            "debug.convergence_landmark_topic");
    convergence_landmark_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrack>(
            convergence_landmark_topic, qos);

    const std::string landmark_pose_topic =
        this->declare_parameter<std::string>("debug.landmark_pose_topic");
    debug_landmark_type_ = static_cast<uint16_t>(
        this->declare_parameter<int>("debug.landmark_type", 0));
    debug_landmark_subtype_ = static_cast<uint16_t>(
        this->declare_parameter<int>("debug.landmark_subtype", 0));
    landmark_pose_debug_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            landmark_pose_topic, qos);
}

void LandmarkServerNode::publish_debug_tracks() {
    vortex_msgs::msg::LandmarkTrackArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;

    for (const auto& t : track_manager_->get_tracks()) {
        vortex_msgs::msg::LandmarkTrack track_msg;

        track_msg.landmark.id = t.id;
        track_msg.landmark.type.value = t.class_key.type;
        track_msg.landmark.subtype.value = t.class_key.subtype;
        track_msg.landmark.header.stamp = this->now();
        track_msg.landmark.header.frame_id = target_frame_;

        const auto pose = t.to_pose();
        track_msg.landmark.pose.pose.position.x = pose.pos_vector().x();
        track_msg.landmark.pose.pose.position.y = pose.pos_vector().y();
        track_msg.landmark.pose.pose.position.z = pose.pos_vector().z();
        track_msg.landmark.pose.pose.orientation.x = pose.ori_quaternion().x();
        track_msg.landmark.pose.pose.orientation.y = pose.ori_quaternion().y();
        track_msg.landmark.pose.pose.orientation.z = pose.ori_quaternion().z();
        track_msg.landmark.pose.pose.orientation.w = pose.ori_quaternion().w();

        track_msg.confirmed = t.confirmed;
        track_msg.hits = t.hits();
        track_msg.misses = t.misses();

        msg.landmark_tracks.push_back(track_msg);
    }

    landmark_track_debug_pub_->publish(msg);
}

void LandmarkServerNode::publish_debug_landmark_pose() {
    for (const auto& t : track_manager_->get_tracks()) {
        if (!t.confirmed)
            continue;
        if (debug_landmark_type_ != 0 &&
            t.class_key.type != debug_landmark_type_)
            continue;
        if (debug_landmark_subtype_ != 0 &&
            t.class_key.subtype != debug_landmark_subtype_)
            continue;

        const auto pose = t.to_pose();
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = target_frame_;
        msg.pose.position.x = pose.pos_vector().x();
        msg.pose.position.y = pose.pos_vector().y();
        msg.pose.position.z = pose.pos_vector().z();
        msg.pose.orientation.x = pose.ori_quaternion().x();
        msg.pose.orientation.y = pose.ori_quaternion().y();
        msg.pose.orientation.z = pose.ori_quaternion().z();
        msg.pose.orientation.w = pose.ori_quaternion().w();
        landmark_pose_debug_pub_->publish(msg);
        return;
    }
}

void LandmarkServerNode::publish_convergence_landmark_debug() {
    if (!convergence_active_ || !convergence_last_known_track_)
        return;

    const auto& track = *convergence_last_known_track_;

    vortex_msgs::msg::LandmarkTrack msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.landmark = track_to_landmark_msg(track);
    msg.confirmed = track.confirmed;
    msg.hits = track.hits();
    msg.misses = track.misses();

    convergence_landmark_debug_pub_->publish(msg);
}

}  // namespace vortex::mission

#include "landmark_server/landmark_server_ros.hpp"
#include <vortex/utils/ros/qos_profiles.hpp>
#include <pose_filtering/ros/pose_filtering_ros_conversions.hpp>

namespace vortex::mission {

void LandmarkServerNode::setup_debug_publishers() {
    std::string debug_topic =
        this->declare_parameter<std::string>("debug.topic_name");
    auto qos = vortex::utils::qos_profiles::sensor_data_profile(10);
    landmark_track_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            debug_topic, qos);

    std::string convergence_landmark_topic =
        this->declare_parameter<std::string>("debug.convergence_landmark_topic");
    convergence_landmark_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::Landmark>(
            convergence_landmark_topic, qos);
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

        track_msg.existence_probability = t.existence_probability;

        msg.landmark_tracks.push_back(track_msg);
    }

    landmark_track_debug_pub_->publish(msg);
}

void LandmarkServerNode::publish_convergence_landmark_debug()
{
    if (!convergence_active_ || !convergence_last_known_track_)
        return;

    const auto& track = *convergence_last_known_track_;

    vortex_msgs::msg::Landmark msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.id = track.id;
    msg.type.value = track.class_key.type;
    msg.subtype.value = track.class_key.subtype;
    msg.pose =
        vortex::filtering::ros_conversions::track_to_pose_with_covariance(track);

    convergence_landmark_debug_pub_->publish(msg);
}

}  // namespace vortex::mission

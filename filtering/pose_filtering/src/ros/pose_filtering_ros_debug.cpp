#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "pose_filtering/ros/pose_filtering_ros.hpp"

namespace vortex::filtering {

void PoseFilteringNode::setup_debug_publishers() {
    std::string meas_topic_name =
        this->declare_parameter<std::string>("debug.topic_name_meas");
    std::string state_topic_name =
        this->declare_parameter<std::string>("debug.topic_name_state");
    const auto qos_sensor_data{
        vortex::utils::qos_profiles::sensor_data_profile(10)};
    pose_meas_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::PoseEulerStamped>(
            meas_topic_name, qos_sensor_data);
    pose_state_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::PoseEulerStamped>(
            state_topic_name, qos_sensor_data);
}

void PoseFilteringNode::publish_meas_debug() {
    if (measurements_.empty()) {
        return;
    }
    Pose meas = measurements_.at(0);

    vortex_msgs::msg::PoseEulerStamped msg;
    msg.header.frame_id = target_frame_;
    msg.header.stamp = this->get_clock()->now();
    msg.x = meas.x;
    msg.y = meas.y;
    msg.z = meas.z;

    Eigen::Vector3d euler =
        vortex::utils::math::quat_to_euler(meas.ori_quaternion());

    msg.roll = euler(0);
    msg.pitch = euler(1);
    msg.yaw = euler(2);

    pose_meas_debug_pub_->publish(msg);
}

void PoseFilteringNode::publish_state_debug() {
    if (track_manager_->get_tracks().empty()) {
        return;
    }
    Track track = track_manager_->get_tracks().at(0);
    Eigen::Vector3d pos = track.state.mean();
    Eigen::Quaterniond quat = track.current_orientation;

    Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(quat);

    vortex_msgs::msg::PoseEulerStamped msg;
    msg.header.frame_id = target_frame_;
    msg.header.stamp = this->get_clock()->now();

    msg.x = pos.x();
    msg.y = pos.y();
    msg.z = pos.z();

    msg.roll = euler(0);
    msg.pitch = euler(1);
    msg.yaw = euler(2);

    pose_state_debug_pub_->publish(msg);
}

}  // namespace vortex::filtering

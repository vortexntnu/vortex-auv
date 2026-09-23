#include "landmark_targets/ros_helpers.hpp"
#include <cmath>
#include <rclcpp/time.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>

namespace vortex::mission {

MapLandmark map_landmark_from_track(
    const vortex_msgs::msg::LandmarkTrack& track) {
    MapLandmark lm;
    lm.id = track.landmark.id;
    lm.pose = vortex::utils::ros_conversions::ros_pose_to_pose(
        track.landmark.pose.pose);
    lm.has_orientation = track.has_orientation;
    lm.last_measurement = rclcpp::Time(track.last_measurement).seconds();
    return lm;
}

std::optional<Eigen::Vector3d> lookup_tool_arm(tf2_ros::Buffer& buffer,
                                               const std::string& base_frame,
                                               const std::string& tool_frame) {
    try {
        const auto tf =
            buffer.lookupTransform(base_frame, tool_frame, tf2::TimePointZero);
        const auto& t = tf.transform.translation;
        return Eigen::Vector3d(t.x, t.y, t.z);
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
}

std::optional<CourseFrame> course_frame_from_tf(
    tf2_ros::Buffer& buffer,
    const std::string& odom_frame,
    const std::string& course_frame_id,
    CourseState state) {
    if (state == CourseState::UNSET) {
        return std::nullopt;
    }
    try {
        const auto tf = buffer.lookupTransform(odom_frame, course_frame_id,
                                               tf2::TimePointZero);
        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;
        CourseFrame course;
        course.origin = Eigen::Vector2d(t.x, t.y);
        course.through_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        course.state = state;
        return course;
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
}

std::optional<CourseFrame> course_frame_from_tf(
    tf2_ros::Buffer& buffer,
    const std::string& odom_frame,
    const std::string& course_frame_id,
    const vortex_msgs::msg::CourseFrameState& state) {
    CourseState s = CourseState::UNSET;
    switch (state.state) {
        case vortex_msgs::msg::CourseFrameState::COARSE:
            s = CourseState::COARSE;
            break;
        case vortex_msgs::msg::CourseFrameState::GATE_LOCKED:
            s = CourseState::GATE_LOCKED;
            break;
        default:
            break;
    }
    return course_frame_from_tf(buffer, odom_frame, course_frame_id, s);
}

}  // namespace vortex::mission

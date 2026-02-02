#include "pose_filtering/ros/pose_filtering_ros_conversions.hpp"

namespace vortex::filtering::ros_conversions {

geometry_msgs::msg::PoseWithCovariance track_to_pose_with_covariance(
    const Track& track) {
    geometry_msgs::msg::PoseWithCovariance msg;

    msg.pose.position.x = track.state_pos.mean()(0);
    msg.pose.position.y = track.state_pos.mean()(1);
    msg.pose.position.z = track.state_pos.mean()(2);

    msg.pose.orientation.w = track.orientation_filter.q.w();
    msg.pose.orientation.x = track.orientation_filter.q.x();
    msg.pose.orientation.y = track.orientation_filter.q.y();
    msg.pose.orientation.z = track.orientation_filter.q.z();

    msg.covariance.fill(0.0);

    const auto& Pp = track.state_pos.cov();
    msg.covariance[0 * 6 + 0] = Pp(0, 0);  // xx
    msg.covariance[0 * 6 + 1] = Pp(0, 1);  // xy
    msg.covariance[0 * 6 + 2] = Pp(0, 2);  // xz

    msg.covariance[1 * 6 + 0] = Pp(1, 0);  // yx
    msg.covariance[1 * 6 + 1] = Pp(1, 1);  // yy
    msg.covariance[1 * 6 + 2] = Pp(1, 2);  // yz

    msg.covariance[2 * 6 + 0] = Pp(2, 0);  // zx
    msg.covariance[2 * 6 + 1] = Pp(2, 1);  // zy
    msg.covariance[2 * 6 + 2] = Pp(2, 2);  // zz

    const auto& Po = track.orientation_filter.error_state.cov();
    msg.covariance[3 * 6 + 3] = Po(0, 0);  // RR
    msg.covariance[3 * 6 + 4] = Po(0, 1);  // RP
    msg.covariance[3 * 6 + 5] = Po(0, 2);  // RY

    msg.covariance[4 * 6 + 3] = Po(1, 0);  // PR
    msg.covariance[4 * 6 + 4] = Po(1, 1);  // PP
    msg.covariance[4 * 6 + 5] = Po(1, 2);  // PY

    msg.covariance[5 * 6 + 3] = Po(2, 0);  // YR
    msg.covariance[5 * 6 + 4] = Po(2, 1);  // YP
    msg.covariance[5 * 6 + 5] = Po(2, 2);  // YY

    return msg;
}

vortex_msgs::msg::LandmarkArray tracks_to_landmark_array_msg(
    const std::vector<Track>& tracks,
    const rclcpp::Time& timestamp,
    const std::string& frame_id) {
    vortex_msgs::msg::LandmarkArray landmark_array_msg;
    landmark_array_msg.header.stamp = timestamp;
    landmark_array_msg.header.frame_id = frame_id;

    for (const auto& track : tracks) {
        vortex_msgs::msg::Landmark landmark;
        landmark.pose = track_to_pose_with_covariance(track);
        landmark_array_msg.landmarks.push_back(landmark);
    }

    return landmark_array_msg;
}

}  // namespace vortex::filtering::ros_conversions

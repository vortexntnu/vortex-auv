#ifndef POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_
#define POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include "pose_filtering/lib/typedefs.hpp"

namespace vortex::filtering::ros_conversions {

/**
 * @brief Convert a Track object to a PoseWithCovariance ROS message.
 * @param track The Track object to convert.
 * @return geometry_msgs::msg::PoseWithCovariance message representing the
 * track.
 */
geometry_msgs::msg::PoseWithCovariance track_to_pose_with_covariance(
    const Track& track);

/**
 * @brief Convert a vector of Track objects to a LandmarkArray ROS message.
 * @param tracks The vector of Track objects to convert.
 * @param timestamp The timestamp to set in the LandmarkArray header.
 * @param frame_id The frame ID to set in the LandmarkArray header.
 * @return vortex_msgs::msg::LandmarkArray message representing the tracks.
 */
vortex_msgs::msg::LandmarkArray tracks_to_landmark_array_msg(
    const std::vector<Track>& tracks,
    const rclcpp::Time& timestamp,
    const std::string& frame_id);

inline Landmark make_landmark_from_pose(const geometry_msgs::msg::Pose& pose,
                                        const LandmarkClassKey& key) {
    Landmark lm;
    lm.pose = vortex::utils::ros_conversions::ros_pose_to_pose(pose);
    lm.class_key = key;
    return lm;
}

inline std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseStamped& msg) {
    return {make_landmark_from_pose(msg.pose,
                                    LandmarkClassKey{.type = 0, .subtype = 0})};
}

inline std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    return {make_landmark_from_pose(msg.pose.pose,
                                    LandmarkClassKey{.type = 0, .subtype = 0})};
}

inline std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseWithCovariance& msg) {
    return {make_landmark_from_pose(msg.pose,
                                    LandmarkClassKey{.type = 0, .subtype = 0})};
}

inline std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::Pose& msg) {
    return {make_landmark_from_pose(msg,
                                    LandmarkClassKey{.type = 0, .subtype = 0})};
}

inline std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseArray& msg) {
    std::vector<Landmark> out;
    out.reserve(msg.poses.size());
    for (const auto& p : msg.poses) {
        out.push_back(make_landmark_from_pose(
            p, LandmarkClassKey{.type = 0, .subtype = 0}));
    }
    return out;
}

inline std::vector<Landmark> ros_to_landmarks(
    const vortex_msgs::msg::LandmarkArray& msg) {
    std::vector<Landmark> out;
    out.reserve(msg.landmarks.size());
    for (const auto& lm_msg : msg.landmarks) {
        Landmark lm;
        lm.pose =
            vortex::utils::ros_conversions::ros_pose_to_pose(lm_msg.pose.pose);
        lm.class_key = LandmarkClassKey{lm_msg.type, lm_msg.subtype};
        out.push_back(lm);
    }
    return out;
}

}  // namespace vortex::filtering::ros_conversions

#endif  // POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_

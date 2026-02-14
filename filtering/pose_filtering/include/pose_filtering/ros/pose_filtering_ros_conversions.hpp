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

/**
 * @brief Create a Landmark object from a Pose message and a class key.
 * @param pose The Pose message to convert.
 * @param key The class key for the landmark.
 * @return Landmark object representing the pose.
 */
Landmark make_landmark_from_pose(const geometry_msgs::msg::Pose& pose,
                                 const LandmarkClassKey& key);

std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseStamped& msg);

std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseWithCovarianceStamped& msg);

std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseWithCovariance& msg);

std::vector<Landmark> ros_to_landmarks(const geometry_msgs::msg::Pose& msg);

std::vector<Landmark> ros_to_landmarks(
    const geometry_msgs::msg::PoseArray& msg);

std::vector<Landmark> ros_to_landmarks(
    const vortex_msgs::msg::LandmarkArray& msg);

}  // namespace vortex::filtering::ros_conversions

#endif  // POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_

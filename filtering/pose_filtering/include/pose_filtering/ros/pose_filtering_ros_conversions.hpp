#ifndef POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_
#define POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
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

}  // namespace vortex::filtering::ros_conversions

#endif  // POSE_FILTERING__ROS__POSE_FILTERING_ROS_CONVERSIONS_HPP_

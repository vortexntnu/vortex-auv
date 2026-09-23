#ifndef LANDMARK_TARGETS__ROS_HELPERS_HPP_
#define LANDMARK_TARGETS__ROS_HELPERS_HPP_

#include <tf2_ros/buffer.h>
#include <optional>
#include <string>
#include <vortex_msgs/msg/course_frame_state.hpp>
#include <vortex_msgs/msg/landmark_track.hpp>
#include "landmark_targets/geometry.hpp"
#include "landmark_targets/landmark_target.hpp"

namespace vortex::mission {

/// Convert a LandmarkTrack from the map to a MapLandmark.
MapLandmark map_landmark_from_track(
    const vortex_msgs::msg::LandmarkTrack& track);

/**
 * @brief Look up the tool arm: the position of @p tool_frame in
 * @p base_frame.
 * @return nullopt if the transform is not available.
 */
std::optional<Eigen::Vector3d> lookup_tool_arm(tf2_ros::Buffer& buffer,
                                               const std::string& base_frame,
                                               const std::string& tool_frame);

/**
 * @brief Read the course frame from TF (landmark_server publishes
 * `course_frame_id` as a child of `odom_frame`).
 *
 * @return nullopt if the frame is not available. landmark_server publishes no
 * TF while the state is UNSET, so no node can compute in a frame that does not
 * exist.
 */
std::optional<CourseFrame> course_frame_from_tf(
    tf2_ros::Buffer& buffer,
    const std::string& odom_frame,
    const std::string& course_frame_id = "nautilus/course",
    CourseState state = CourseState::COARSE);

/// Same, with the state taken from a CourseFrameState message. Returns
/// nullopt if the state is UNSET or the TF is missing.
std::optional<CourseFrame> course_frame_from_tf(
    tf2_ros::Buffer& buffer,
    const std::string& odom_frame,
    const std::string& course_frame_id,
    const vortex_msgs::msg::CourseFrameState& state);

}  // namespace vortex::mission

#endif  // LANDMARK_TARGETS__ROS_HELPERS_HPP_

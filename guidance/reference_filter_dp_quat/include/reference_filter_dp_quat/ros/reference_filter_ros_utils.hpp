#ifndef REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_
#define REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/reference_filter_quat.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"
#include "reference_filter_dp_quat/lib/waypoint_types.hpp"

namespace vortex::guidance {

/// @brief Convert a ROS waypoint mode to a WaypointMode enum.
/// @throws std::invalid_argument if the mode value is not recognized.
inline WaypointMode waypoint_mode_from_ros(uint8_t mode) {
    switch (mode) {
        case vortex_msgs::msg::Waypoint::FULL_POSE:
            return WaypointMode::FULL_POSE;
        case vortex_msgs::msg::Waypoint::ONLY_POSITION:
            return WaypointMode::ONLY_POSITION;
        case vortex_msgs::msg::Waypoint::FORWARD_HEADING:
            return WaypointMode::FORWARD_HEADING;
        case vortex_msgs::msg::Waypoint::ONLY_ORIENTATION:
            return WaypointMode::ONLY_ORIENTATION;
        default:
            throw std::invalid_argument("Invalid ROS waypoint mode: " +
                                        std::to_string(mode));
    }
}

/// @brief Convert a ROS Waypoint message to an internal Waypoint struct.
inline vortex::guidance::Waypoint waypoint_from_ros(
    const vortex_msgs::msg::Waypoint& ros_wp) {
    Waypoint wp;
    wp.pose = vortex::utils::ros_conversions::ros_pose_to_pose(ros_wp.pose);
    wp.mode = waypoint_mode_from_ros(ros_wp.mode);
    return wp;
}

/// @brief Fill a ReferenceFilterQuat message from a Pose and velocity vector.
inline vortex_msgs::msg::ReferenceFilterQuat fill_reference_msg(
    const vortex::utils::types::Pose& pose,
    const Eigen::Vector6d& velocity) {
    vortex_msgs::msg::ReferenceFilterQuat msg;
    msg.x = pose.x;
    msg.y = pose.y;
    msg.z = pose.z;
    msg.qw = pose.qw;
    msg.qx = pose.qx;
    msg.qy = pose.qy;
    msg.qz = pose.qz;
    msg.x_dot = velocity(0);
    msg.y_dot = velocity(1);
    msg.z_dot = velocity(2);
    msg.roll_dot = velocity(3);
    msg.pitch_dot = velocity(4);
    msg.yaw_dot = velocity(5);
    return msg;
}

/// @brief Fill an RPY ReferenceFilter message from a Pose and velocity vector.
inline vortex_msgs::msg::ReferenceFilter fill_reference_rpy_msg(
    const vortex::utils::types::Pose& pose,
    const Eigen::Vector6d& velocity) {
    using vortex::utils::math::ssa;
    auto euler = pose.as_pose_euler();
    vortex_msgs::msg::ReferenceFilter msg;
    msg.x = euler.x;
    msg.y = euler.y;
    msg.z = euler.z;
    msg.roll = ssa(euler.roll);
    msg.pitch = ssa(euler.pitch);
    msg.yaw = ssa(euler.yaw);
    msg.x_dot = velocity(0);
    msg.y_dot = velocity(1);
    msg.z_dot = velocity(2);
    msg.roll_dot = velocity(3);
    msg.pitch_dot = velocity(4);
    msg.yaw_dot = velocity(5);
    return msg;
}

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_

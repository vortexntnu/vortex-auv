#ifndef REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_
#define REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>
#include "reference_filter_dp/lib/eigen_typedefs.hpp"
#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

/**
 * @brief Convert a ROS waypoint mode to a WaypointMode enum.
 * @throws std::invalid_argument if the mode value is not recognized.
 */
inline WaypointMode waypoint_mode_from_ros(
    const vortex_msgs::msg::WaypointMode& mode_msg) {
    switch (mode_msg.mode) {
        case vortex_msgs::msg::WaypointMode::FULL_POSE:
            return WaypointMode::FULL_POSE;
        case vortex_msgs::msg::WaypointMode::ONLY_POSITION:
            return WaypointMode::ONLY_POSITION;
        case vortex_msgs::msg::WaypointMode::FORWARD_HEADING:
            return WaypointMode::FORWARD_HEADING;
        case vortex_msgs::msg::WaypointMode::ONLY_ORIENTATION:
            return WaypointMode::ONLY_ORIENTATION;
        default:
            throw std::invalid_argument("Invalid ROS waypoint mode: " +
                                        std::to_string(mode_msg.mode));
    }
}

/// @brief Convert a ROS Waypoint message to an internal Waypoint struct.
inline vortex::guidance::Waypoint waypoint_from_ros(
    const vortex_msgs::msg::Waypoint& ros_wp) {
    Waypoint wp;
    wp.pose =
        vortex::utils::ros_conversions::ros_pose_to_pose_euler(ros_wp.pose);
    wp.mode = waypoint_mode_from_ros(ros_wp.waypoint_mode);
    return wp;
}

/// @brief Fill a ReferenceFilter message from an 18D state vector.
inline vortex_msgs::msg::ReferenceFilter fill_reference_msg(
    const Eigen::Vector18d& x) {
    using vortex::utils::math::ssa;
    vortex_msgs::msg::ReferenceFilter msg;
    msg.x = x(0);
    msg.y = x(1);
    msg.z = x(2);
    msg.roll = ssa(x(3));
    msg.pitch = ssa(x(4));
    msg.yaw = ssa(x(5));
    msg.x_dot = x(6);
    msg.y_dot = x(7);
    msg.z_dot = x(8);
    msg.roll_dot = x(9);
    msg.pitch_dot = x(10);
    msg.yaw_dot = x(11);
    return msg;
}

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_

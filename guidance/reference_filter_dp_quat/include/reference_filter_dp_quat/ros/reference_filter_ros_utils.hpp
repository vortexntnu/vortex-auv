#ifndef REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_
#define REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_UTILS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/reference_filter_quat.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"

namespace vortex::guidance {

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

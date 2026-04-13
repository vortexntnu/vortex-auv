#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

types::Eta eta_convert_from_ros_to_eigen(
    const geometry_msgs::msg::PoseWithCovariance& msg) {
    types::Eta eta;
    eta.x = msg.pose.position.x;
    eta.y = msg.pose.position.y;
    eta.z = msg.pose.position.z;
    eta.qw = msg.pose.orientation.w;
    eta.qx = msg.pose.orientation.x;
    eta.qy = msg.pose.orientation.y;
    eta.qz = msg.pose.orientation.z;

    return eta;
}

types::Nu nu_convert_from_ros_to_eigen(
    const geometry_msgs::msg::TwistWithCovariance& msg) {
    types::Nu nu;
    nu.u = msg.twist.linear.x;
    nu.v = msg.twist.linear.y;
    nu.w = msg.twist.linear.z;
    nu.p = msg.twist.angular.x;
    nu.q = msg.twist.angular.y;
    nu.r = msg.twist.angular.z;

    return nu;
}

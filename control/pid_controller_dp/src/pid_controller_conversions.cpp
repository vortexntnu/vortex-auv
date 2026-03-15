#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

types::Eta eta_convert_from_ros_to_eigen(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    types::Eta eta;
    eta.x = msg->pose.pose.position.x;
    eta.y = msg->pose.pose.position.y;
    eta.z = msg->pose.pose.position.z;

    eta.qw = msg->pose.pose.orientation.w;
    eta.qx = msg->pose.pose.orientation.x;
    eta.qy = msg->pose.pose.orientation.y;
    eta.qz = msg->pose.pose.orientation.z;

    return eta;
}

types::Nu nu_convert_from_ros_to_eigen(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    types::Nu nu;
    nu.u = msg->twist.twist.linear.x;
    nu.v = msg->twist.twist.linear.y;
    nu.w = msg->twist.twist.linear.z;

    nu.p = msg->twist.twist.angular.x;
    nu.q = msg->twist.twist.angular.y;
    nu.r = msg->twist.twist.angular.z;

    return nu;
}

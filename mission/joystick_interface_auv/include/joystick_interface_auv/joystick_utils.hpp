#ifndef STATE_HPP
#define STATE_HPP

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick_interface_auv {
struct State {
    double x : 0.0;
    double y : 0.0;
    double z : 0.0;
    double roll : 0.0;
    double pitch : 0.0;
    double yaw : 0.0;

    void from_pose(const& geometry_msgs::msg::PoseWithCovarianceStamped pose) {
        x = pose.pose.pose.position.x;
        y = pose.pose.pose.position.y;
        z = pose.pose.pose.position.z;
        Eigen::Quaterniond q(
            pose.pose.pose.orientation.w, pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
        Eigen::Vector3d euler_angles = quat_to_euler(q);
        roll = euler_angles[0];
        pitch = euler_angles[1];
        yaw = euler_angles[2];
    }

    void from_
}

Eigen::Vector3d
quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler_angles;
}

}  // namespace joystick_interface_auv

#endif  // STATE_HPP

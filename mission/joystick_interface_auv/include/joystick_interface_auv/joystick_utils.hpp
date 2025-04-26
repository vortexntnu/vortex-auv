#ifndef STATE_HPP
#define STATE_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/StdVector"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick_interface_auv {
struct State {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    void from_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
        x = pose.pose.pose.position.x;
        y = pose.pose.pose.position.y;
        z = pose.pose.pose.position.z;
        Eigen::Quaternion<double> q(
            pose.pose.pose.orientation.w, pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
        Eigen::Vector3d euler_angles = quat_to_euler(q);
        roll = euler_angles[0];
        pitch = euler_angles[1];
        yaw = euler_angles[2];
    }

    void from_twist(const geometry_msgs::msg::Twist& twist) {
        x = twist.linear.x;
        y = twist.linear.y;
        z = twist.linear.z;
        roll = twist.angular.x;
        pitch = twist.angular.y;
        yaw = twist.angular.z;
    }

}

Eigen::Vector3d
quat_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
    return euler_angles;
}

struct JoyStates {
    std::string XBOX_MODE = "xbox";
    std::string AUTONOMOUS_MODE = "autonomous";
    std::string KILLSWITCH = "killswitch";
    std::string REFERENCE_MODE = "reference";
}

struct Wired {
    std::unordered_map<std::string, int> joystick_buttons_map{
        {"A", 0},
        {"B", 1},
        {"X", 2},
        {"Y", 3},
        {"LB", 4},
        {"RB", 5},
        {"back", 6},
        {"start", 7},
        {"power", 8},
        {"stick_button_left", 9},
        {"stick_button_right", 10},
        {"share_button", 11}};

    std::unordered_map<std::string, int> joystick_axes_map{
        {"horizontal_axis_left_stick", 0},
        {"vertical_axis_left_stick", 1},
        {"LT", 2},
        {"horizontal_axis_right_stick", 3},
        {"vertical_axis_right_stick", 4},
        {"RT", 5},
        {"dpad_horizontal", 6},
        {"dpad_vertical", 7}};
};

struct WirelessXboxSeriesX {
    std::unordered_map<std::string, int> joystick_buttons_map{
        {"A", 0},
        {"B", 1},
        {"0", 2},
        {"X", 3},
        {"Y", 4},
        {"0", 5},
        {"LB", 6},
        {"RB", 7},
        {"0", 8},
        {"0", 9},
        {"back", 10},
        {"start", 11},
        {"power", 12},
        {"stick_button_left", 13},
        {"stick_button_right", 14},
        {"share_button", 15}};

    std::unordered_map<std::string, int> joystick_axes_map{
        {"horizontal_axis_left_stick", 0},
        {"vertical_axis_left_stick", 1},
        {"horizontal_axis_right_stick", 2},
        {"vertical_axis_right_stick", 3},
        {"RT", 4},
        {"LT", 5},
        {"dpad_horizontal", 6},
        {"dpad_vertical", 7}};
};

}  // namespace joystick_interface_auv

#endif  // STATE_HPP

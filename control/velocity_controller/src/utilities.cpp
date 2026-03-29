#include "velocity_controller/utilities.hpp"
#include <std_msgs/msg  //string.h>
#include <casadi/casadi.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <stdexcept>
#include "Eigen/Dense"

angle quaternion_to_euler_angle(double w, double x, double y, double z) {
    double ysqr = y * y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    double phi = std::atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double theta = std::asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);
    double psi = std::atan2(t3, t4);

    return {phi, theta, psi};
};

State State::operator=(nav_msgs::msg::Odometry::SharedPtr rhs) {
    w = rhs->pose.pose.orientation.w;
    x = rhs->pose.pose.orientation.x;
    y = rhs->pose.pose.orientation.y;
    z = rhs->pose.pose.orientation.z;

    auto [r, p, y_] = quaternion_to_euler_angle(w, x, y, z);
    roll = r;
    pitch = p;
    yaw = y_;

    // angular velocity
    roll_rate = rhs->twist.twist.angular.x;
    pitch_rate = rhs->twist.twist.angular.y;
    yaw_rate = rhs->twist.twist.angular.z;
    // velocity
    surge = rhs->twist.twist.linear.x;
    sway = rhs->twist.twist.linear.y;
    heave = rhs->twist.twist.linear.z;

    return (*this);
}

geometry_msgs::msg::Quaternion euler_angle_to_quaternion(double roll,
                                                         double pitch,
                                                         double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

angle angle_NED_to_body(double roll_des,
                        double pitch_des,
                        double yaw_des,
                        double roll,
                        double pitch,
                        double yaw) {
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw), sy = std::sin(yaw);

    // R_current: NED to body for current attitude
    Eigen::Matrix3d R_current;
    R_current << cp * cy, cp * sy, -sp, sr * sp * cy - cr * sy,
        sr * sp * sy + cr * cy, sr * cp, cr * sp * cy + sr * sy,
        cr * sp * sy - sr * cy, cr * cp;

    double cr_d = std::cos(roll_des), sr_d = std::sin(roll_des);
    double cp_d = std::cos(pitch_des), sp_d = std::sin(pitch_des);
    double cy_d = std::cos(yaw_des), sy_d = std::sin(yaw_des);

    // R_desired: NED to body for desired attitude
    Eigen::Matrix3d R_desired;
    R_desired << cp_d * cy_d, cp_d * sy_d, -sp_d,
        sr_d * sp_d * cy_d - cr_d * sy_d, sr_d * sp_d * sy_d + cr_d * cy_d,
        sr_d * cp_d, cr_d * sp_d * cy_d + sr_d * sy_d,
        cr_d * sp_d * sy_d - sr_d * cy_d, cr_d * cp_d;

    // R_error = R_desired * R_current^T
    Eigen::Matrix3d R_error = R_desired * R_current.transpose();

    // Extract euler angles from R_error — this gives the error in body frame
    double pitch_err = std::asin(-R_error(2, 0));
    double roll_err = std::atan2(R_error(2, 1), R_error(2, 2));
    double yaw_err = std::atan2(R_error(1, 0), R_error(0, 0));

    return {roll_err, pitch_err, yaw_err};
}

angle State::get_angle() {
    return {roll, pitch, yaw};
}
Guidance_data& Guidance_data::operator=(
    const vortex_msgs::msg::LOSGuidance::SharedPtr& msg) {
    surge = msg->surge;
    pitch = msg->pitch;
    yaw = msg->yaw;
    return *this;
}

#ifndef VELOCITY_NODE_TEST_ACCESSOR_HPP_
#define VELOCITY_NODE_TEST_ACCESSOR_HPP_
#include "velocity_controller/velocity_controller_ros.hpp"
#include "rclcpp/rclcpp.hpp"

class VelocityNodeTestAccessor {
public:
    static void guidance_callback(Velocity_node& node,
                                   vortex_msgs::msg::LOSGuidance::SharedPtr msg) {
        node.guidance_callback(msg);
    }
    static void odometry_callback(Velocity_node& node,
                                   nav_msgs::msg::Odometry::SharedPtr msg) {
        node.odometry_callback(msg);
    }
    static void publish_thrust(Velocity_node& node) {
        node.publish_thrust();
    }
    static const Guidance_data& get_guidance_values(const Velocity_node& node) {
        return node.guidance_values;
    }
    static int get_publish_counter(const Velocity_node& node) {
        return node.publish_counter;
    }
    static const control_manager* get_control_manager(const Velocity_node& node) {
        return node.control_manager_ptr.get();
    }
};
#endif
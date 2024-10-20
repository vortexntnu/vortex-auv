#ifndef THRUSTER_INTERFACE_AUV_ROS_HPP
#define THRUSTER_INTERFACE_AUV_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include "thruster_interface_auv_cpp/thruster_interface_auv_driver.hpp"

class ThrusterInterfaceAUVNode : public rclcpp::Node {
public:
    explicit ThrusterInterfaceAUVNode();

private:
    void thruster_forces_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr thruster_forces_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr thruster_pwm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int> thruster_mapping_;
    std::vector<int> thruster_direction_;
    std::vector<int> thruster_pwm_offset_;
    std::vector<int> pwm_min_;
    std::vector<int> pwm_max_;
    std::vector<double> thruster_forces_array_;
    double thrust_timer_period_;
    
    ThrusterInterfaceAUVDriver thruster_driver_;
};

#endif // THRUSTER_INTERFACE_AUV_ROS_HPP

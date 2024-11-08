#ifndef THRUSTER_INTERFACE_AUV_NODE_HPP
#define THRUSTER_INTERFACE_AUV_NODE_HPP

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include "thruster_interface_auv/thruster_interface_auv_driver.hpp"

class ThrusterInterfaceAUVNode : public rclcpp::Node {
   public:
    ThrusterInterfaceAUVNode();

   private:
    void thruster_forces_callback(
        const vortex_msgs::msg::ThrusterForces::SharedPtr msg);
    void timer_callback();
    void extract_all_parameters();

    int i2c_bus_;
    int i2c_address_;
    std::string subscriber_topic_name_;
    std::string publisher_topic_name_;
    std::vector<ThrusterParameters> thruster_parameters_;
    std::vector<std::vector<double>> poly_coeffs_;

    std::vector<double> thruster_forces_array_;
    double thrust_timer_period_;

    std::unique_ptr<ThrusterInterfaceAUVDriver> thruster_driver_;
    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr
        thruster_forces_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr
        thruster_pwm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // THRUSTER_INTERFACE_AUV_NODE_HPP

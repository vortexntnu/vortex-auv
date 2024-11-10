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
    /**
     * @brief periodically receive thruster forces topic
     *
     * @param msg ThrusterForces message
     */
    void thruster_forces_callback(
        const vortex_msgs::msg::ThrusterForces::SharedPtr msg);

    /**
     * @brief publish and send pwm commands to thrusters. Sinchronous with
     * thruster_forces_callback
     */
    void pwm_callback();

    /**
     * @brief extract all parameters from the .yaml file
     */
    void extract_all_parameters();

    int i2c_bus_;
    int i2c_address_;
    std::string subscriber_topic_name_;
    std::string publisher_topic_name_;
    std::vector<ThrusterParameters> thruster_parameters_;
    std::vector<std::vector<double>> poly_coeffs_;

    std::vector<double> thruster_forces_array_;
    bool debug_flag_;

    std::unique_ptr<ThrusterInterfaceAUVDriver>
        thruster_driver_;  ///<-- pwm driver
    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::
        SharedPtr  ///<-- thruster forces subscriber
            thruster_forces_subscriber_;
    rclcpp::Publisher<
        std_msgs::msg::Int16MultiArray>::SharedPtr  ///<-- pwm publisher
        thruster_pwm_publisher_;
};

#endif  // THRUSTER_INTERFACE_AUV_NODE_HPP

#ifndef THRUSTER_INTERFACE_AUV_NODE_HPP
#define THRUSTER_INTERFACE_AUV_NODE_HPP

#include <rclcpp/parameter_event_handler.hpp>
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
     * @brief watchdog callback to check if thruster forces are being received
     */
    void watchdog_callback();

    /**
     * @brief extract all parameters from the .yaml file
     */
    void extract_all_parameters();

    /**
     * @brief Initialize the parameter handler and a parameter event callback.
     *
     */
    void initialize_parameter_handler();

    /**
     * @brief create/reinitialize publisher if flag=1, kill it if flag=0
     */
    void set_publisher();

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
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_msg_time_;
    rclcpp::Duration watchdog_timeout_ = std::chrono::seconds(1);
    bool watchdog_triggered_ = false;

    /**
     * @brief Manages parameter events for the node.
     *
     * This handler is used to set up a mechanism to listen for and react to
     * changes in parameters via terminal at runtime.
     */
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;

    /**
     * @brief Handle to the registration of the parameter event callback.
     *
     * Represents a token or reference to the specific callback registration
     * made with the parameter event handler (`param_handler_`).
     */
    rclcpp::ParameterCallbackHandle::SharedPtr debug_flag_parameter_cb;

    /**
     * specific callback for updating debug_flag.
     */
    void update_debug_flag(const rclcpp::Parameter& p);
};

#endif  // THRUSTER_INTERFACE_AUV_NODE_HPP

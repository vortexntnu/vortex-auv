#ifndef PID_CONTROLLER_DP__PID_CONTROLLER_ROS_HPP_
#define PID_CONTROLLER_DP__PID_CONTROLLER_ROS_HPP_

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <variant>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/msg/operation_mode.hpp>
#include <vortex_msgs/msg/reference_filter_quat.hpp>
#include <vortex_msgs/srv/get_operation_mode.hpp>
#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/typedefs.hpp"

// @brief Class for the PID controller node
class PIDControllerNode : public rclcpp::Node {
   public:
    explicit PIDControllerNode(const rclcpp::NodeOptions & options);

   private:
    // @brief Callback function for the killswitch topic
    // @param msg: Bool message containing the killswitch status
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // @brief Callback function for the software mode topic
    // @param msg: String message containing the software mode
    void operation_mode_callback(
        const vortex_msgs::msg::OperationMode::SharedPtr msg);

    // @brief Callback function for the tau publisher timer
    void publish_tau();

    // @brief Set the PID controller parameters
    void set_pid_params();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Initialize the operation mode by calling the GetOperationMode
    // service
    void initialize_operation_mode();

    // @brief Callback function for the guidance topic
    // @param msg: ReferenceFilter message containing the desired vehicle pose
    // and velocity
    void guidance_callback(
        const vortex_msgs::msg::ReferenceFilterQuat::SharedPtr msg);

    // @brief Callback function for the odometry topic
    // @param msg: Odometry message containing the AUV pose and speed
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // @brief Callback function for parameter updates
    // @param parameters: vector of parameters to be set
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    rclcpp::Client<vortex_msgs::srv::GetOperationMode>::SharedPtr
        get_operation_mode_client_;

    PIDController pid_controller_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killswitch_sub_;

    rclcpp::Subscription<vortex_msgs::msg::OperationMode>::SharedPtr
        operation_mode_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<vortex_msgs::msg::ReferenceFilterQuat>::SharedPtr
        guidance_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr kp_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ki_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr kd_sub_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_pub_;

    rclcpp::TimerBase::SharedPtr tau_pub_timer_;

    std::chrono::milliseconds time_step_;

    types::Eta eta_;

    types::Eta eta_d_;

    types::Nu nu_;

    types::Eta eta_dot_d_;

    bool killswitch_on_{true};

    vortex::utils::types::Mode operation_mode_{
        vortex::utils::types::Mode::manual};

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

#endif  // PID_CONTROLLER_DP__PID_CONTROLLER_ROS_HPP_

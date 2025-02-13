#ifndef DP_ADAPT_BACKS_CONTROLLER_ROS_HPP
#define DP_ADAPT_BACKS_CONTROLLER_ROS_HPP

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <variant>
#include <vortex_msgs/msg/reference_filter.hpp>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// @brief struct for the topics loaded from the parameter server
struct Topics {
    std::string dp_reference_topic;
    std::string pose_topic;
    std::string twist_topic;
    std::string software_kill_switch_topic;
    std::string software_operation_mode_topic;
    std::string control_topic;
};

// @brief Class for the DP Adaptive Backstepping controller node
class DPAdaptBacksControllerNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit DPAdaptBacksControllerNode();

    // @brief function for configuring the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for activating the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for cleaning up after the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for deactivating the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state);

    // @brief function for shutting down the node
    // @param previous_state: lifecycle::State previous state of the node
    LifecycleCallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& previous_state);

   private:
    // @brief Callback function for the killswitch topic
    // @param msg: Bool message containing the killswitch status
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // @brief Callback function for the software mode topic
    // @param msg: String message containing the software mode
    void software_mode_callback(const std_msgs::msg::String::SharedPtr msg);

    // @brief Callback function for the pose topic
    // @param msg: PoseWithCovarianceStamped message containing the AUV pose
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // @brief Callback function for the twist topic
    // @param msg: TwistWithCovarianceStamped message containing the AUV speed
    void twist_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // @brief Callback function for the control input tau publish
    void publish_tau();

    // @brief set the DP Adaptive Backstepping controller parameters
    void set_adap_params();

    // @brief Callback function for the guidance topic
    // @param msg: ReferenceFilter message containing the desired vehicle pose
    // and velocity
    void guidance_callback(
        const vortex_msgs::msg::ReferenceFilter::SharedPtr msg);

    // @brief get the topics from the parameter server
    void get_topics(Topics& topics);

    DPAdaptBacksController dp_adapt_backs_controller_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killswitch_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr software_mode_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    rclcpp::Subscription<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        guidance_sub_;

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Wrench>::SharedPtr
        tau_pub_;

    rclcpp::TimerBase::SharedPtr tau_pub_timer_;

    std::chrono::milliseconds time_step_;

    dp_types::Eta eta_;

    dp_types::Eta eta_d_;

    dp_types::Nu nu_;

    bool killswitch_on_;

    std::string software_mode_;
};

#endif

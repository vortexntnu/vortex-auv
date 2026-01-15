#ifndef LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_
#define LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/los_guidance.hpp>
#include <vortex_msgs/msg/los_guidance.hpp>
#include <vortex_msgs/msg/waypoints.hpp>
#include <vortex_msgs/srv/set_los_mode.hpp>
#include "los_guidance/lib/adaptive_los.hpp"
#include "los_guidance/lib/integral_los.hpp"
#include "los_guidance/lib/proportional_los.hpp" 
#include "los_guidance/lib/vector_field_los.hpp"
#include "los_guidance/lib/types.hpp" 
#include <vortex/utils/math.hpp>
#include <string>
#include <vortex_msgs/msg/pose_euler_stamped.hpp>

namespace vortex::guidance::los {

class LosGuidanceNode : public rclcpp::Node {
   public:
    LosGuidanceNode();

   private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher(YAML::Node config);

    // @brief Set the action server
    void set_action_server();

    // @brief Determine the LOS mode service
    void set_service_server();

    // @brief Set the adaptive LOS guidance parameters
    void set_adaptive_los_guidance(YAML::Node config);

    // @brief Set the proportional LOS guidance parameters
    void set_proportional_los_guidance(YAML::Node config);

    // @brief Set the integral LOS guidance parameters
    void set_integral_los_guidance(YAML::Node config);

    // @brief Set the vector field LOS guidance parameters
    void set_vector_field_guidance(YAML::Node config);

    // @brief Callback for the waypoint topic
    // @param msg The reference message
    void waypoint_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // @brief Callback for the pose topic
    // @param msg The pose message
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

    // @brief Handle the goal request
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal);

    // @brief Handle the cancel request
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
            goal_handle);

    // @brief Handle the accepted request
    // @param goal_handle The goal handle
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                             vortex_msgs::action::LOSGuidance>> goal_handle);

    // @brief Execute the goal
    // @param goal_handle The goal handle
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::LOSGuidance>> goal_handle);

    void set_los_mode(
        const std::shared_ptr<vortex_msgs::srv::SetLosMode::Request> request,
        std::shared_ptr<vortex_msgs::srv::SetLosMode::Response> response);

    void publish_state_debug(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        current_pose);

    vortex_msgs::msg::LOSGuidance fill_los_reference(types::Outputs output);

    YAML::Node get_los_config(std::string yaml_file_path);

    void parse_common_config(YAML::Node common_config);

    rclcpp_action::Server<vortex_msgs::action::LOSGuidance>::SharedPtr
        action_server_;

    rclcpp::Service<vortex_msgs::srv::SetLosMode>::SharedPtr los_mode_service_;

    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr reference_pub_;

    bool enable_debug_;

    std::string debug_topic_name_;

    rclcpp::Publisher<vortex_msgs::msg::PoseEulerStamped>::SharedPtr debug_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        waypoint_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::TimerBase::SharedPtr reference_pub_timer_;

    std::chrono::milliseconds time_step_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    types::Inputs path_inputs_{};

    double u_desired_{};

    double goal_reached_tol_{};

    std::unique_ptr<AdaptiveLOSGuidance> adaptive_los_{};
    std::unique_ptr<IntegralLOSGuidance> integral_los_{};
    std::unique_ptr<ProportionalLOSGuidance> proportional_los_{};
    std::unique_ptr<VectorFieldLOSGuidance> vector_field_los_{};
    types::ActiveLosMethod method_{};
};

}  // namespace vortex::guidance::los
 
#endif  // LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_

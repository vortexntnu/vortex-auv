#ifndef LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_
#define LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex/utils/math.hpp>
#include <vortex_msgs/action/los_guidance.hpp>
#include <vortex_msgs/msg/detail/los_guidance__struct.hpp>
#include <vortex_msgs/msg/los_guidance.hpp>
#include <vortex_msgs/msg/waypoints.hpp>
#include <vortex_msgs/srv/set_los_mode.hpp>

#include <memory>
#include <string>

#include "los_guidance/lib/adaptive_los.hpp"
#include "los_guidance/lib/integral_los.hpp"
#include "los_guidance/lib/proportional_los.hpp"
#include "los_guidance/lib/types.hpp"
#include "los_guidance/lib/vector_field_los.hpp"

namespace vortex::guidance::los {

// LOS Guidance ROS Node
class LosGuidanceNode : public rclcpp::Node {
   public:
    // Constructor
    LosGuidanceNode();

   private:
    // Setup Functions
    void set_subscribers_and_publisher();
    void set_action_server();
    void set_service_server();

    // Configuration Functions
    void set_adaptive_los_guidance(YAML::Node config);
    void set_proportional_los_guidance(YAML::Node config);
    void set_integral_los_guidance(YAML::Node config);
    void set_vector_field_guidance(YAML::Node config);
    YAML::Node get_los_config(std::string yaml_file_path);
    void parse_common_config(YAML::Node common_config);

    // Callback Functions
    void waypoint_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Action Server Functions
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
            goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                             vortex_msgs::action::LOSGuidance>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::LOSGuidance>> goal_handle);

    // Service Functions
    void set_los_mode(
        const std::shared_ptr<vortex_msgs::srv::SetLosMode::Request> request,
        std::shared_ptr<vortex_msgs::srv::SetLosMode::Response> response);

    // Publish Functions
    void publish_state_debug(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
            current_pose);
    vortex_msgs::msg::LOSGuidance fill_los_reference(types::Outputs output,
                                                     types::Inputs inputs);

    // State Flags
    bool has_active_segment_{false};

    // ROS Interfaces
    rclcpp_action::Server<vortex_msgs::action::LOSGuidance>::SharedPtr
        action_server_;
    rclcpp::Service<vortex_msgs::srv::SetLosMode>::SharedPtr los_mode_service_;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr reference_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr los_debug_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr
        state_debug_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        waypoint_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr reference_pub_timer_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // Timing and Synchronization
    std::chrono::milliseconds time_step_;
    std::mutex mutex_;

    // Action State
    rclcpp_action::GoalUUID preempted_goal_id_;
    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle_;

    // Guidance State
    types::Inputs path_inputs_{};
    double u_desired_{};
    double goal_reached_tol_{};
    double max_pitch_angle_{};
    bool slow_approach_{};
    double slow_down_distance_{};
    bool surge_initialized_{false};
    double u_slow_min_{};
    double commanded_surge_{};
    double surge_rate_limit_{};
    types::ActiveLosMethod method_{};

    // Guidance Modules
    std::unique_ptr<AdaptiveLOSGuidance> adaptive_los_{};
    std::unique_ptr<IntegralLOSGuidance> integral_los_{};
    std::unique_ptr<ProportionalLOSGuidance> proportional_los_{};
    std::unique_ptr<VectorFieldLOSGuidance> vector_field_los_{};

    // Debug Data
    nav_msgs::msg::Odometry::SharedPtr debug_current_odom_{};
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_

/**
 * @file los_guidance_ros.hpp
 * @brief The LosGuidanceNode class initializes ROS interfaces, loads
 * configuration parameters, and runs the LOS guidance node.
 */
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

/**
 * @brief The LosGuidanceNode class initializes ROS interfaces, loads LOS
 * guidance parameters, and manages path-following execution.
 */
class LosGuidanceNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructs a LosGuidanceNode object.
     * @param options ROS node options used when creating the node.
     */
    explicit LosGuidanceNode(const rclcpp::NodeOptions& options);

   private:
    /**
     * @brief Sets up the ROS subscribers and publishers used by the node.
     */
    void set_subscribers_and_publisher();

    /**
     * @brief Sets up the LOS guidance action server.
     */
    void set_action_server();

    /**
     * @brief Sets up the service server used for changing LOS guidance mode.
     */
    void set_service_server();

    /**
     * @brief Initializes the adaptive LOS guidance module from configuration.
     * @param config YAML configuration node containing adaptive LOS parameters.
     */
    void set_adaptive_los_guidance(YAML::Node config);

    /**
     * @brief Initializes the proportional LOS guidance module from
     * configuration.
     * @param config YAML configuration node containing proportional LOS
     * parameters.
     */
    void set_proportional_los_guidance(YAML::Node config);

    /**
     * @brief Initializes the integral LOS guidance module from configuration.
     * @param config YAML configuration node containing integral LOS parameters.
     */
    void set_integral_los_guidance(YAML::Node config);

    /**
     * @brief Initializes the vector field LOS guidance module from
     * configuration.
     * @param config YAML configuration node containing vector field LOS
     * parameters.
     */
    void set_vector_field_guidance(YAML::Node config);

    /**
     * @brief Loads the LOS guidance YAML configuration file.
     * @param yaml_file_path Path to the YAML configuration file.
     * @return YAML::Node Parsed YAML configuration.
     */
    YAML::Node get_los_config(std::string yaml_file_path);

    /**
     * @brief Parses common guidance parameters shared by all LOS methods.
     * @param common_config YAML node containing common guidance parameters.
     */
    void parse_common_config(YAML::Node common_config);

    /**
     * @brief Callback for receiving waypoint updates.
     * @param msg Received waypoint message.
     */
    void waypoint_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /**
     * @brief Callback for receiving pose updates.
     * @param msg Received pose message.
     */
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Callback for receiving odometry updates.
     * @param msg Received odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Handles an incoming LOS guidance action goal request.
     * @param uuid Unique identifier for the received goal.
     * @param goal Requested LOS guidance goal.
     * @return rclcpp_action::GoalResponse Response indicating whether the goal
     * is accepted.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal);

    /**
     * @brief Handles cancellation of an active LOS guidance goal.
     * @param goal_handle Handle to the goal being cancelled.
     * @return rclcpp_action::CancelResponse Response indicating whether
     * cancellation is accepted.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
            goal_handle);

    /**
     * @brief Handles an accepted LOS guidance goal.
     * @param goal_handle Handle to the accepted goal.
     */
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                             vortex_msgs::action::LOSGuidance>> goal_handle);

    /**
     * @brief Executes the LOS guidance action.
     * @param goal_handle Handle to the active LOS guidance goal.
     */
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::LOSGuidance>> goal_handle);

    /**
     * @brief Service callback for changing the active LOS guidance method.
     * @param request Service request containing the desired LOS mode.
     * @param response Service response indicating whether the mode change
     * succeeded.
     */
    void set_los_mode(
        const std::shared_ptr<vortex_msgs::srv::SetLosMode::Request> request,
        std::shared_ptr<vortex_msgs::srv::SetLosMode::Response> response);

    /**
     * @brief Publishes debug information about the current vehicle state.
     * @param current_pose Current vehicle pose used for debug publishing.
     */
    void publish_state_debug(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
            current_pose);

    /**
     * @brief Fills a LOS guidance reference message from computed outputs and
     * inputs.
     * @param output Calculated LOS guidance outputs.
     * @param inputs Current LOS guidance inputs.
     * @return vortex_msgs::msg::LOSGuidance Populated LOS guidance reference
     * message.
     */
    vortex_msgs::msg::LOSGuidance fill_los_reference(types::Outputs output);

    /**
     * @brief Checks if the given LOS guidance goal is feasible based on the
     * provided inputs.
     * @param inputs Current LOS guidance inputs.
     * @return true if the goal is feasible, false otherwise.
     */
    bool is_goal_feasible(
        const types::Inputs& inputs,
        std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal);

    /**
     * @brief Checks if the LOS guidance goal has been missed based on the
     * provided inputs.
     * @param inputs Current LOS guidance inputs.
     * @return true if the goal is missed, false otherwise.
     */
    bool is_goal_missed(const types::Inputs& inputs);

    bool has_active_segment_{false};

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

    std::chrono::milliseconds time_step_;
    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;
    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle_;

    types::Inputs path_inputs_{};
    double u_desired_{};
    double goal_reached_tol_{};
    double max_pitch_angle_{};
    types::ActiveLosMethod method_{};

    double nearest_been_to_goal_{std::numeric_limits<double>::max()};
    double time_since_nearest_goal_{};
    double missed_goal_distance_margin_{};
    double missed_goal_timeout_{};

    std::unique_ptr<AdaptiveLOSGuidance> adaptive_los_{};
    std::unique_ptr<IntegralLOSGuidance> integral_los_{};
    std::unique_ptr<ProportionalLOSGuidance> proportional_los_{};
    std::unique_ptr<VectorFieldLOSGuidance> vector_field_los_{};

    nav_msgs::msg::Odometry::SharedPtr debug_current_odom_{};
};

}  // namespace vortex::guidance::los

#endif  // LOS_GUIDANCE__LOS_GUIDANCE_ROS_HPP_

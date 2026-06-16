#ifndef REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_
#define REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vortex_msgs/msg/waypoint_debug.hpp>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/action/guidance_waypoint.hpp>
#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/msg/reference_filter_quat.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"

namespace vortex::guidance {

enum class DebugPublishMode { none, timer, on_new_goal };

class ReferenceFilterNode : public rclcpp::Node {
   public:
    explicit ReferenceFilterNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~ReferenceFilterNode();

   private:
    void set_subscribers_and_publisher();
    void set_action_server();
    void set_refererence_filter();

    void setup_reset_subscription();
    void setup_debug_publisher();
    void publish_debug_goal();
    void publish_waypoint_goal(const vortex::utils::types::Waypoint& wp,
                               double convergence_threshold);

    void on_system_reset(std_msgs::msg::Empty::ConstSharedPtr msg);

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
            goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>> goal_handle);

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                     vortex_msgs::action::GuidanceWaypoint>> goal_handle,
                 bool retarget);

    rclcpp_action::Server<vortex_msgs::action::GuidanceWaypoint>::SharedPtr
        action_server_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;

    ReferenceFilterParams filter_params_;

    std::unique_ptr<WaypointFollower> follower_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilterQuat>::SharedPtr
        reference_pub_;

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        rpy_debug_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        debug_goal_pub_;

    rclcpp::Publisher<vortex_msgs::msg::WaypointDebug>::SharedPtr
        waypoint_goal_pub_;

    bool publish_rpy_debug_{false};
    DebugPublishMode debug_mode_{DebugPublishMode::none};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

    rclcpp::Subscription<vortex_msgs::msg::DVLAltitude>::SharedPtr
        altitude_sub_;

    std::chrono::milliseconds time_step_{};

    vortex::utils::types::Pose current_pose_;

    vortex::utils::types::Twist current_twist_;

    bool altitude_control_enabled_{false};
    double current_altitude_{0.0};
    bool altitude_valid_{false};
    double altitude_lp_alpha_{0.9};

    std::mutex sensor_mutex_;

    std::atomic<bool> preempted_{false};
    std::atomic<bool> executing_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__ROS__REFERENCE_FILTER_ROS_HPP_

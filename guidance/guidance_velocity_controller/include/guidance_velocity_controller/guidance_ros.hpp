#pragma once
#include <memory>
#include <functional>
#include <mutex>
#include <cmath>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <spdlog/spdlog.h>

#include <guidance_velocity_controller/guidance.hpp>

#include <vortex_msgs/action/los_guidance.hpp>
#include <vortex_msgs/msg/los_guidance.hpp>
#include <vortex_msgs/msg/waypoints.hpp>

class GuidanceNode : public rclcpp::Node {
public:
  explicit GuidanceNode();

private:
  // init
  void establish_communications();
  void setup_parameters();

  // io
  void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publish_setpoints();
  vortex_msgs::msg::LOSGuidance fill_los_reference();
  void fill_los_waypoints(const geometry_msgs::msg::PointStamped & g_waypoints);

  // actions
  using GoalHandle = rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>;
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute_guidance(const std::shared_ptr<GoalHandle> goal_handle);

  // pubs/subs
  rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr refrence_out;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_in;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_in;
  rclcpp_action::Server<vortex_msgs::action::LOSGuidance>::SharedPtr guidance_action_server;

  // guidance
  std::unique_ptr<LOSGuidance> adaptive_los_guidance_;

  // state
  LOS::Point current_position_{};
  LOS::Point last_waypoint_{};
  LOS::Point target_waypoint_{};

  std::mutex mutex_;
  rclcpp_action::GoalUUID preempted_goal_id_{};
  std::shared_ptr<GoalHandle> goal_handle_{};

  // timing
  std::chrono::milliseconds time_step{0};

  // outputs
  double yaw_d_;
  double pitch_d_;
  double u_d_ ;
};

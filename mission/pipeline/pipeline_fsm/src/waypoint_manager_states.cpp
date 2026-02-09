#include "pipeline_fsm/waypoint_manager_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "vortex_msgs/msg/waypoint.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace pipeline_fsm {

StartWaypointManagerState::StartWaypointManagerState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}) {}

std::string StartWaypointManagerState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  YASMIN_LOG_INFO("Starting Waypoint Manager (persistent mode)");

  cancelled_ = false;

  auto node = yasmin_ros::YasminNode::get_instance();
  auto client = rclcpp_action::create_client<WaypointManagerAction>(
      node, "waypoint_manager");

  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    YASMIN_LOG_WARN("Waypoint Manager action server not available - continuing anyway (mock mode)");
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  auto goal_msg = WaypointManagerAction::Goal();
  goal_msg.persistent = true;
  goal_msg.convergence_threshold = 0.5;

  // Add dummy waypoints for testing (hover in place)
  vortex_msgs::msg::Waypoint wp;
  wp.pose.position.x = 0.0;
  wp.pose.position.y = 0.0;
  wp.pose.position.z = -2.0;
  wp.pose.orientation.w = 1.0;
  goal_msg.waypoints.push_back(wp);

  YASMIN_LOG_INFO("Sending waypoint manager goal with 1 dummy waypoint");

  auto send_goal_options =
      rclcpp_action::Client<WaypointManagerAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandle> goal_handle) {
        if (!goal_handle) {
          YASMIN_LOG_WARN("Waypoint Manager goal rejected");
        } else {
          YASMIN_LOG_INFO("Waypoint Manager goal accepted");
          goal_handle_ = goal_handle;
        }
      };

  client->async_send_goal(goal_msg, send_goal_options);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  if (cancelled_) {
    return yasmin_ros::basic_outcomes::ABORT;
  }

  return yasmin_ros::basic_outcomes::SUCCEED;
}

void StartWaypointManagerState::cancel_state() {
  YASMIN_LOG_INFO("Cancelling StartWaypointManagerState");
  cancelled_ = true;

  if (goal_handle_) {
    auto node = yasmin_ros::YasminNode::get_instance();
    auto client = rclcpp_action::create_client<WaypointManagerAction>(
        node, "waypoint_manager");
    client->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

StopWaypointManagerState::StopWaypointManagerState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

std::string StopWaypointManagerState::execute(
    yasmin::Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  YASMIN_LOG_INFO("Stopping Waypoint Manager");

  auto node = yasmin_ros::YasminNode::get_instance();
  using WaypointManagerAction = vortex_msgs::action::WaypointManager;
  auto client = rclcpp_action::create_client<WaypointManagerAction>(
      node, "waypoint_manager");

  if (client->wait_for_action_server(std::chrono::seconds(2))) {
    client->async_cancel_all_goals();
  }

  return yasmin_ros::basic_outcomes::SUCCEED;
}

}

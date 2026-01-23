#include "mission_execution/start_waypoint_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace mission_execution
{

StartWaypointAction::StartWaypointAction(
  const std::string& name,
  const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
: BT::StatefulActionNode(name, config),
  node_(params.nh),
  action_name_("/orca/waypoint_manager")
{
  auto node = node_.lock();
  if (!node)
    throw std::runtime_error("StartWaypointAction: ROS node not available");

  client_ =
    rclcpp_action::create_client<vortex_msgs::action::WaypointManager>(
      node, action_name_);
}

BT::NodeStatus StartWaypointAction::onStart()
{
  goal_sent_ = false;
  goal_accepted_ = false;

  auto node = node_.lock();
  if (!node)
    return BT::NodeStatus::FAILURE;

  if (auto input = getInput<std::string>("action_name"))
  {
    if (input.value() != action_name_)
    {
      action_name_ = input.value();
      client_ =
        rclcpp_action::create_client<vortex_msgs::action::WaypointManager>(
          node, action_name_);
    }
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(logger(), "WaypointManager action server not available");
    return BT::NodeStatus::FAILURE;
  }

  vortex_msgs::action::WaypointManager::Goal goal;
  goal.waypoints.clear();
  goal.convergence_threshold = 0.5;
  goal.persistent = true;

  auto send_opts =
    rclcpp_action::Client<vortex_msgs::action::WaypointManager>::SendGoalOptions();

  send_opts.goal_response_callback =
    [this](auto future)
    {
      auto handle = future.get();
      if (handle)
      {
        goal_accepted_ = true;
        RCLCPP_INFO(logger(), "StartWaypointAction: Persistent waypoint action accepted");
      }
      else
      {
        RCLCPP_ERROR(logger(), "StartWaypointAction: Goal rejected");
      }
    };

  client_->async_send_goal(goal, send_opts);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StartWaypointAction::onRunning()
{
  if (!goal_sent_)
    return BT::NodeStatus::FAILURE;

  if (goal_accepted_)
    return BT::NodeStatus::SUCCESS;

  return BT::NodeStatus::RUNNING;
}

void StartWaypointAction::onHalted()
{
  goal_sent_ = false;
  goal_accepted_ = false;
}

}  // namespace mission_execution

#include "mission_execution/stop_waypoint_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace mission_execution
{

StopWaypointAction::StopWaypointAction(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
  , node_(params.nh)
  , action_name_("/orca/waypoint_manager")
  , cancellation_started_(false)
{
  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error("StopWaypointAction: ROS node is not available");
  }

  // Get action name from port (will be read in onStart)
  // Create action client
  action_client_ = rclcpp_action::create_client<vortex_msgs::action::WaypointManager>(
    node, action_name_);
}

BT::NodeStatus StopWaypointAction::onStart()
{
  cancellation_started_ = false;
  
  auto node = node_.lock();
  if (!node)
  {
    return BT::NodeStatus::FAILURE;
  }

  // Get action name from port
  if (auto action_name_input = getInput<std::string>("action_name"))
  {
    std::string new_action_name = action_name_input.value();
    if (new_action_name != action_name_)
    {
      action_name_ = new_action_name;
      // Recreate action client with new name
      action_client_ = rclcpp_action::create_client<vortex_msgs::action::WaypointManager>(
        node, action_name_);
    }
  }

  // Check if action server is available
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_WARN(logger(), "StopWaypointAction: Action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // Cancel all goals for this action
  // We store the future to check it in onRunning
  cancel_future_ = action_client_->async_cancel_all_goals();
  
  cancellation_started_ = true;
  RCLCPP_INFO(logger(), "StopWaypointAction: Cancelling all waypoint actions...");
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopWaypointAction::onRunning()
{
  if (!cancellation_started_)
  {
    return BT::NodeStatus::FAILURE;
  }

  // Check if cancellation is complete
  if (cancel_future_.valid() && 
      cancel_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    try 
    {
      auto cancel_response = cancel_future_.get();
      // action_msgs/srv/CancelGoal Response has 'goals_canceling' list
      RCLCPP_INFO(logger(), "StopWaypointAction: Waypoint action stopped (%zu goals cancelled)", cancel_response->goals_canceling.size());
      return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger(), "StopWaypointAction: Exception checking cancel result: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  
  return BT::NodeStatus::RUNNING;
}

void StopWaypointAction::onHalted()
{
  cancellation_started_ = false;
  RCLCPP_INFO(logger(), "StopWaypointAction: Halted");
}

}  // namespace mission_execution


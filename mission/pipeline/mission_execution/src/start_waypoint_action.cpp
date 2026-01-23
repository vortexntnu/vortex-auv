#include "mission_execution/start_waypoint_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace mission_execution
{

StartWaypointAction::StartWaypointAction(
  const std::string& name, const BT::NodeConfig& conf,
  const BT::RosNodeParams& params)
  : BT::RosActionNode<vortex_msgs::action::WaypointManager>(name, conf, params)
{
}

bool StartWaypointAction::setGoal(Goal& goal)
{
  // Start WaypointAction with persistent=true and empty waypoints
  // This allows external nodes to send waypoints continuously
  goal.waypoints.clear();
  goal.convergence_threshold = 0.5;
  goal.persistent = true;  // Critical: keeps action active to accept external waypoints

  RCLCPP_INFO(logger(), "StartWaypointAction: Starting persistent waypoint action");
  return true;
}

BT::NodeStatus StartWaypointAction::onResultReceived(const WrappedResult& /*wr*/)
{
  // This should not normally be called since persistent=true keeps action running
  // But if it is, we consider it success
  RCLCPP_INFO(logger(), "StartWaypointAction: Waypoint action started successfully");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StartWaypointAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "StartWaypointAction: Failed to start waypoint action: %s",
               BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace mission_execution


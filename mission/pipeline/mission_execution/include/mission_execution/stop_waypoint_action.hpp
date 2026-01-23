#ifndef MISSION_EXECUTION__STOP_WAYPOINT_ACTION_HPP_
#define MISSION_EXECUTION__STOP_WAYPOINT_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "vortex_msgs/action/waypoint_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

namespace mission_execution
{

class StopWaypointAction : public BT::StatefulActionNode
{
public:
  StopWaypointAction(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name", "/orca/waypoint_manager", "Action server name")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::weak_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<vortex_msgs::action::WaypointManager>::SharedPtr action_client_;
  std::string action_name_;
  bool cancellation_started_;
  std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal::Response>> cancel_future_;  

  rclcpp::Logger logger()
  {
    if (auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("StopWaypointAction");
  }
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__STOP_WAYPOINT_ACTION_HPP_


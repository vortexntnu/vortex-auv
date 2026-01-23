#ifndef MISSION_EXECUTION__START_WAYPOINT_ACTION_HPP_
#define MISSION_EXECUTION__START_WAYPOINT_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "vortex_msgs/action/waypoint_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

namespace mission_execution
{

class StartWaypointAction : public BT::StatefulActionNode
{
public:
  StartWaypointAction(const std::string& name,
                      const BT::NodeConfig& config,
                      const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name",
                                "/orca/waypoint_manager",
                                "WaypointManager action name")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::weak_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<vortex_msgs::action::WaypointManager>::SharedPtr client_;

  std::string action_name_;
  bool goal_sent_{false};
  bool goal_accepted_{false};

  rclcpp::Logger logger() const
  {
    if (auto node = node_.lock())
      return node->get_logger();
    return rclcpp::get_logger("StartWaypointAction");
  }
};

}  // namespace mission_execution

#endif

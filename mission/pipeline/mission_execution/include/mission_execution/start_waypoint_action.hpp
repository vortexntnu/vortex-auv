#ifndef MISSION_EXECUTION__START_WAYPOINT_ACTION_HPP_
#define MISSION_EXECUTION__START_WAYPOINT_ACTION_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "vortex_msgs/action/waypoint_manager.hpp"

namespace mission_execution
{

class StartWaypointAction : public BT::RosActionNode<vortex_msgs::action::WaypointManager>
{
public:
  StartWaypointAction(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("action_name", "/orca/waypoint_manager", "Action server name")
    });
  }

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__START_WAYPOINT_ACTION_HPP_


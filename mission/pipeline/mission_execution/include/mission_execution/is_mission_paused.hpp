#ifndef MISSION_EXECUTION__IS_MISSION_PAUSED_HPP_
#define MISSION_EXECUTION__IS_MISSION_PAUSED_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace mission_execution
{

class IsMissionPaused : public BT::ConditionNode
{
public:
  IsMissionPaused(const std::string& name,
                  const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "exec_state",
        "Execution state from MissionExecutionGate"
      )
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__IS_MISSION_PAUSED_HPP_

#ifndef MISSION_EXECUTION__SET_MISSION_PHASE_HPP_
#define MISSION_EXECUTION__SET_MISSION_PHASE_HPP_

#include "behaviortree_cpp/action_node.h"

namespace mission_execution
{

class SetMissionPhase : public BT::SyncActionNode
{
public:
  SetMissionPhase(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("value", "Mission phase value (SEARCH, CONVERGE, PIPELINE, DONE)"),
      BT::OutputPort<std::string>("mission_phase_request", "Internal port to request phase alignment")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__SET_MISSION_PHASE_HPP_


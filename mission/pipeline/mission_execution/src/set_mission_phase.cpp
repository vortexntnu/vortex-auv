#include "mission_execution/set_mission_phase.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace mission_execution
{

SetMissionPhase::SetMissionPhase(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus SetMissionPhase::tick()
{
  auto phase_value = getInput<std::string>("value");
  if (!phase_value)
  {
    return BT::NodeStatus::FAILURE;
  }

  // Write to blackboard with the key that MissionExecutionGate reads
  setOutput("mission_phase_request", phase_value.value());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mission_execution


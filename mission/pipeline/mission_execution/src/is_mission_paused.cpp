#include "mission_execution/is_mission_paused.hpp"

namespace mission_execution
{

IsMissionPaused::IsMissionPaused(const std::string& name,
                                 const BT::NodeConfig& config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsMissionPaused::tick()
{
  auto state = getInput<std::string>("exec_state");
  if (!state)
  {
    return BT::NodeStatus::FAILURE;
  }

  return (state.value() == "PAUSED")
           ? BT::NodeStatus::SUCCESS
           : BT::NodeStatus::FAILURE;
}

}  // namespace mission_execution

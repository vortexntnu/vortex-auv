// This file ensures all plugins are registered when the library is loaded

#include "mission_execution/mission_execution_gate.hpp"
#include "mission_execution/set_mission_phase.hpp"
#include "mission_execution/start_waypoint_action.hpp"
#include "mission_execution/stop_waypoint_action.hpp"
#include "mission_execution/landmark_enum_polling.hpp"
#include "mission_execution/landmark_converge.hpp"
#include "mission_execution/search_pattern.hpp"
#include "mission_execution/pipeline_following.hpp"
#include "mission_execution/is_mission_paused.hpp"

#include "behaviortree_ros2/plugins.hpp"


BT_REGISTER_ROS_NODES(factory, params)
{
 
  factory.registerNodeType<mission_execution::MissionExecutionGate>(
    "MissionExecutionGate", params);

  factory.registerNodeType<mission_execution::SetMissionPhase>(
    "SetMissionPhase");

  
  factory.registerNodeType<mission_execution::StartWaypointAction>(
    "StartWaypointAction", params);
  factory.registerNodeType<mission_execution::StopWaypointAction>(
    "StopWaypointAction", params);

  
  factory.registerNodeType<mission_execution::LandmarkEnumPolling>(
    "LandmarkEnumPolling");
  factory.registerNodeType<mission_execution::LandmarkConverge>(
    "LandmarkConverge");
  factory.registerNodeType<mission_execution::SearchPattern>(
    "SearchPattern", params);
  factory.registerNodeType<mission_execution::PipelineFollowing>(
    "PipelineFollowing", params);

  
  factory.registerNodeType<mission_execution::IsMissionPaused>(
    "IsMissionPaused");
}

// This file ensures all plugins are registered when the library is loaded
#include "mission_execution/mission_execution_gate.hpp"
#include "mission_execution/set_mission_phase.hpp"
#include "mission_execution/start_waypoint_action.hpp"
#include "mission_execution/stop_waypoint_action.hpp"
#include "mission_execution/landmark_enum_polling.hpp"
#include "mission_execution/landmark_converge.hpp"
#include "mission_execution/search_pattern.hpp"
#include "mission_execution/pipeline_following.hpp"
#include "behaviortree_ros2/plugins.hpp"

// Register all nodes
BT_REGISTER_ROS_NODES(factory, params)
{
  // MissionExecutionGate needs ROS node for services
  factory.registerNodeType<mission_execution::MissionExecutionGate>("MissionExecutionGate", params);
  
  // SetMissionPhase is a simple SyncActionNode, no ROS needed
  factory.registerNodeType<mission_execution::SetMissionPhase>("SetMissionPhase");
  
  // StartWaypointAction needs ROS for action client
  factory.registerNodeType<mission_execution::StartWaypointAction>("StartWaypointAction", params);
  
  // StopWaypointAction needs ROS for action client (to cancel goals)
  factory.registerNodeType<mission_execution::StopWaypointAction>("StopWaypointAction", params);
  
  // LandmarkEnumPolling and LandmarkConverge are now SyncActionNodes
  factory.registerNodeType<mission_execution::LandmarkEnumPolling>("LandmarkEnumPolling");
  factory.registerNodeType<mission_execution::LandmarkConverge>("LandmarkConverge");
  
  // SearchPattern and PipelineFollowing need ROS for service clients
  factory.registerNodeType<mission_execution::SearchPattern>("SearchPattern", params);
  factory.registerNodeType<mission_execution::PipelineFollowing>("PipelineFollowing", params);

}


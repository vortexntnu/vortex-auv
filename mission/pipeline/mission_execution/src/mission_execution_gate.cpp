#include "mission_execution/mission_execution_gate.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <rclcpp/rclcpp.hpp>

namespace mission_execution
{

MissionExecutionGate::MissionExecutionGate(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::ConditionNode(name, config)
  , node_(params.nh)
  , exec_state_(ExecState::IDLE)
  , mission_phase_(MissionPhase::SEARCH)
  , mission_id_("")
{
  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error("MissionExecutionGate: ROS node is not available");
  }

  // Create services
  // Create services
  mission_control_service_ = node->create_service<vortex_msgs::srv::MissionControl>(
    "/mission/control",
    [this](const std::shared_ptr<vortex_msgs::srv::MissionControl::Request> request,
           std::shared_ptr<vortex_msgs::srv::MissionControl::Response> response) {
      missionControlCallback(request, response);
    });

  RCLCPP_INFO(logger(), "MissionExecutionGate: Services initialized");
}

BT::NodeStatus MissionExecutionGate::tick()
{

  // Check for phase update request from blackboard (from SetMissionPhase)
  auto phase_request = getInput<std::string>("mission_phase_request");
  if (phase_request)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    MissionPhase requested_phase = stringToPhase(phase_request.value());
    if (requested_phase != mission_phase_)
    {
      mission_phase_ = requested_phase;
      RCLCPP_INFO(logger(), "MissionExecutionGate: Phase updated to %s",
                  phaseToString(mission_phase_).c_str());
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Write state to blackboard
    setOutput("mission_phase", phaseToString(mission_phase_));
    setOutput("mission_id", mission_id_);

    // Return status based on execution state
    // IMPORTANT: Return RUNNING for IDLE and PAUSED to keep tree ticking
    // Tree only terminates when explicitly aborted (abort service resets to IDLE then calls haltTree)
    if (exec_state_ == ExecState::IDLE)
    {
      return BT::NodeStatus::RUNNING;  // Keep ticking, wait for mission/start
    }
    else if (exec_state_ == ExecState::PAUSED)
    {
      return BT::NodeStatus::RUNNING;  // Keep ticking but blocked
    }
    else  // RUNNING
    {
      return BT::NodeStatus::SUCCESS;  // Allow tree to progress
    }
  }
}

void MissionExecutionGate::missionControlCallback(
  const std::shared_ptr<vortex_msgs::srv::MissionControl::Request> request,
  std::shared_ptr<vortex_msgs::srv::MissionControl::Response> response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  switch (request->command)
  {
    case vortex_msgs::srv::MissionControl::Request::START:
      if (exec_state_ != ExecState::IDLE)
      {
        response->success = false;
        response->message = "Mission already started";
        RCLCPP_WARN(logger(), "MissionExecutionGate: Start requested but mission already active");
        return;
      }

      exec_state_ = ExecState::RUNNING;
      mission_phase_ = MissionPhase::SEARCH;
      mission_id_ = request->mission_id.empty() ? "default" : request->mission_id;

      response->success = true;
      response->message = "Mission started";
      RCLCPP_INFO(logger(), "MissionExecutionGate: Mission started (ID: %s)", mission_id_.c_str());
      break;

    case vortex_msgs::srv::MissionControl::Request::PAUSE:
      if (exec_state_ != ExecState::RUNNING)
      {
        response->success = false;
        response->message = "Mission is not running";
        RCLCPP_WARN(logger(), "MissionExecutionGate: Pause requested but mission not running");
        return;
      }

      exec_state_ = ExecState::PAUSED;
      response->success = true;
      response->message = "Mission paused";
      RCLCPP_INFO(logger(), "MissionExecutionGate: Mission paused (phase: %s)",
                  phaseToString(mission_phase_).c_str());
      break;

    case vortex_msgs::srv::MissionControl::Request::RESUME:
      if (exec_state_ != ExecState::PAUSED)
      {
        response->success = false;
        response->message = "Mission is not paused";
        RCLCPP_WARN(logger(), "MissionExecutionGate: Resume requested but mission not paused");
        return;
      }

      exec_state_ = ExecState::RUNNING;
      // Do NOT reset mission_phase - resume from same phase
      response->success = true;
      response->message = "Mission resumed";
      RCLCPP_INFO(logger(), "MissionExecutionGate: Mission resumed (phase: %s)",
                  phaseToString(mission_phase_).c_str());
      break;

    case vortex_msgs::srv::MissionControl::Request::ABORT:
      exec_state_ = ExecState::IDLE;
      mission_phase_ = MissionPhase::SEARCH;
      mission_id_ = "";

      response->success = true;
      response->message = "Mission aborted";
      RCLCPP_INFO(logger(), "MissionExecutionGate: Mission aborted");
      break;

    default:
      response->success = false;
      response->message = "Unknown command";
      RCLCPP_ERROR(logger(), "MissionExecutionGate: Unknown command received: %d", request->command);
      break;
  }
}

std::string MissionExecutionGate::phaseToString(MissionPhase phase) const
{
  switch (phase)
  {
    case MissionPhase::SEARCH:
      return "SEARCH";
    case MissionPhase::CONVERGE:
      return "CONVERGE";
    case MissionPhase::PIPELINE:
      return "PIPELINE";
    case MissionPhase::DONE:
      return "DONE";
    default:
      return "UNKNOWN";
  }
}

MissionPhase MissionExecutionGate::stringToPhase(const std::string& phase_str) const
{
  if (phase_str == "SEARCH")
    return MissionPhase::SEARCH;
  else if (phase_str == "CONVERGE")
    return MissionPhase::CONVERGE;
  else if (phase_str == "PIPELINE")
    return MissionPhase::PIPELINE;
  else if (phase_str == "DONE")
    return MissionPhase::DONE;
  else
    return MissionPhase::SEARCH;  // Default
}

}  // namespace mission_execution


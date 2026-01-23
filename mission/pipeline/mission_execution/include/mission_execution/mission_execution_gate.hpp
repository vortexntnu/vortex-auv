#ifndef MISSION_EXECUTION__MISSION_EXECUTION_GATE_HPP_
#define MISSION_EXECUTION__MISSION_EXECUTION_GATE_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "vortex_msgs/srv/mission_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <mutex>
#include <memory>
#include <string>

namespace mission_execution
{

enum class ExecState
{
  IDLE,
  RUNNING,
  PAUSED
};

enum class MissionPhase
{
  SEARCH,
  CONVERGE,
  PIPELINE,
  DONE
};

class MissionExecutionGate : public BT::ConditionNode
{
public:
  MissionExecutionGate(const std::string& name, const BT::NodeConfig& config,
                       const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mission_phase_request", "", "Requested mission phase (from SetMissionPhase)"),
      BT::OutputPort<std::string>("exec_state", "Execution state: IDLE | RUNNING | PAUSED"),
      BT::OutputPort<std::string>("mission_phase", "Current mission phase"),
      BT::OutputPort<std::string>("mission_id", "Current mission ID")
    };
  }

  BT::NodeStatus tick() override;

private:
  void missionControlCallback(
    const std::shared_ptr<vortex_msgs::srv::MissionControl::Request> request,
    std::shared_ptr<vortex_msgs::srv::MissionControl::Response> response);

  std::string phaseToString(MissionPhase phase) const;
  MissionPhase stringToPhase(const std::string& phase_str) const;

  std::weak_ptr<rclcpp::Node> node_;
  
  rclcpp::Service<vortex_msgs::srv::MissionControl>::SharedPtr mission_control_service_;

  std::mutex state_mutex_;
  ExecState exec_state_;
  MissionPhase mission_phase_;
  std::string mission_id_;

  rclcpp::Logger logger()
  {
    if (auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("MissionExecutionGate");
  }
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__MISSION_EXECUTION_GATE_HPP_


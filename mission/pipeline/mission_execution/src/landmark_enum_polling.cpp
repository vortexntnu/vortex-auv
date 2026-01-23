#include "mission_execution/landmark_enum_polling.hpp"
#include <chrono>
#include <thread>
#include <iostream>

namespace mission_execution
{

LandmarkEnumPolling::LandmarkEnumPolling(
  const std::string& name, const BT::NodeConfig& conf)
  : BT::StatefulActionNode(name, conf)
{
}

BT::NodeStatus LandmarkEnumPolling::onStart()
{
  uint8_t landmark_type = 1;
  if (auto type_input = getInput<uint8_t>("landmark_type"))
  {
    landmark_type = type_input.value();
  }

  double duration = 5.0;
  if (auto duration_input = getInput<double>("duration"))
  {
    duration = duration_input.value();
  }

  std::cout << "LandmarkEnumPolling: Scanning for landmark type " << static_cast<int>(landmark_type) 
            << ". Waiting for " << duration << " seconds..." << std::endl;
  
  start_time_ = std::chrono::system_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandmarkEnumPolling::onRunning()
{
  double duration = 5.0;
  if (auto duration_input = getInput<double>("duration"))
  {
    duration = duration_input.value();
  }

  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now() - start_time_).count();

  if (elapsed >= duration)
  {
    // Mock results
    int32_t mock_landmark_id = 1;
    geometry_msgs::msg::Pose mock_pose;
    mock_pose.position.x = 5.0;
    mock_pose.position.y = 3.0;
    mock_pose.position.z = -2.0;
    mock_pose.orientation.w = 1.0;

    setOutput("landmark_id", mock_landmark_id);
    setOutput("landmark_pose", mock_pose);

    std::cout << "LandmarkEnumPolling: Found landmark ID " << mock_landmark_id << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace mission_execution


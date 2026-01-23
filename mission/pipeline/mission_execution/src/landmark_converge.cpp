#include "mission_execution/landmark_converge.hpp"
#include <chrono>
#include <thread>

namespace mission_execution
{

LandmarkConverge::LandmarkConverge(
  const std::string& name, const BT::NodeConfig& conf)
  : BT::StatefulActionNode(name, conf)
{
}

BT::NodeStatus LandmarkConverge::onStart()
{
  auto landmark_id_input = getInput<int32_t>("landmark_id");
  int32_t landmark_id = landmark_id_input ? landmark_id_input.value() : -1;

  double duration = 5.0;
  if (auto duration_input = getInput<double>("duration"))
  {
    duration = duration_input.value();
  }

  std::cout << "LandmarkConverge: Converging to landmark ID " << landmark_id 
            << ". Waiting for " << duration << " seconds..." << std::endl;

  start_time_ = std::chrono::system_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandmarkConverge::onRunning()
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
    auto landmark_id_input = getInput<int32_t>("landmark_id");
    int32_t landmark_id = landmark_id_input ? landmark_id_input.value() : -1;
    std::cout << "LandmarkConverge: Successfully converged to landmark ID " << landmark_id << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

}  // namespace mission_execution


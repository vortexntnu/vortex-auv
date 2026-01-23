#ifndef MISSION_EXECUTION__LANDMARK_CONVERGE_HPP_
#define MISSION_EXECUTION__LANDMARK_CONVERGE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission_execution
{

class LandmarkConverge : public BT::StatefulActionNode
{
public:
  LandmarkConverge(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int32_t>("landmark_id", "ID of the landmark to converge to"),
      BT::InputPort<geometry_msgs::msg::Pose>("landmark_pose", "Pose of the landmark"),
      BT::InputPort<double>("convergence_threshold", 0.5, "Convergence threshold"),
      BT::InputPort<double>("duration", 5.0, "Mock duration in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};

private:
  std::chrono::system_clock::time_point start_time_;
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__LANDMARK_CONVERGE_HPP_


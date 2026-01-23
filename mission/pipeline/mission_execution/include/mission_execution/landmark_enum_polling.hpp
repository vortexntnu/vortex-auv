#ifndef MISSION_EXECUTION__LANDMARK_ENUM_POLLING_HPP_
#define MISSION_EXECUTION__LANDMARK_ENUM_POLLING_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission_execution
{

class LandmarkEnumPolling : public BT::StatefulActionNode
{
public:
  LandmarkEnumPolling(const std::string& name, const BT::NodeConfig& conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint8_t>("landmark_type", 1, "Landmark type to enumerate (BUOY=1, BOAT=2, WALL=69)"),
      BT::InputPort<double>("duration", 5.0, "Mock duration in seconds"),
      BT::OutputPort<int32_t>("landmark_id", "ID of the detected landmark"),
      BT::OutputPort<geometry_msgs::msg::Pose>("landmark_pose", "Pose of the landmark")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};

private:
  std::chrono::system_clock::time_point start_time_;
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__LANDMARK_ENUM_POLLING_HPP_


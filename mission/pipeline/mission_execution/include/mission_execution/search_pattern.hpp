#ifndef MISSION_EXECUTION__SEARCH_PATTERN_HPP_
#define MISSION_EXECUTION__SEARCH_PATTERN_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"
#include "vortex_msgs/srv/send_waypoints.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mission_execution
{

class SearchPattern : public BT::StatefulActionNode
{
public:
  SearchPattern(const std::string& name, const BT::NodeConfig& conf,
               const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("start_pose", "Starting pose for search pattern"),
      BT::InputPort<double>("square_size", 10.0, "Size of the square in meters"),
      BT::InputPort<double>("convergence_threshold", 0.5, "Convergence threshold for waypoint"),
      BT::InputPort<bool>("persistent", true, "Whether the action should be persistent"),
      BT::InputPort<double>("duration", 60.0, "Mock execution duration in seconds"),
      BT::InputPort<std::string>("service_name", "/orca/waypoint_addition", "Waypoint addition service name")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr client_;
  std::shared_future<vortex_msgs::srv::SendWaypoints::Response::SharedPtr> future_;
  std::chrono::system_clock::time_point start_time_;
  bool waypoints_sent_;

  std::vector<geometry_msgs::msg::Pose> generateSquareWaypoints(
    const geometry_msgs::msg::Pose& start_pose,
    double square_size);
};

}  // namespace mission_execution

#endif  // MISSION_EXECUTION__SEARCH_PATTERN_HPP_


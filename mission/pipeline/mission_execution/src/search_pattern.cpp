#include "mission_execution/search_pattern.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace mission_execution
{

SearchPattern::SearchPattern(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, conf), node_(params.nh), waypoints_sent_(false)
{
  client_ = node_->create_client<vortex_msgs::srv::SendWaypoints>("/orca/waypoint_addition");
}

std::vector<geometry_msgs::msg::Pose> SearchPattern::generateSquareWaypoints(
  const geometry_msgs::msg::Pose& start_pose,
  double square_size)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Waypoint 1: Move forward (positive X)
  geometry_msgs::msg::Pose wp1 = start_pose;
  wp1.position.x += square_size;
  waypoints.push_back(wp1);
  
  // Waypoint 2: Move right (positive Y)
  geometry_msgs::msg::Pose wp2 = wp1;
  wp2.position.y += square_size;
  waypoints.push_back(wp2);
  
  // Waypoint 3: Move back (negative X)
  geometry_msgs::msg::Pose wp3 = wp2;
  wp3.position.x -= square_size;
  waypoints.push_back(wp3);
  
  // Waypoint 4: Return to start (negative Y)
  geometry_msgs::msg::Pose wp4 = wp3;
  wp4.position.y -= square_size;
  waypoints.push_back(wp4);
  
  return waypoints;
}

BT::NodeStatus SearchPattern::onStart()
{
  waypoints_sent_ = false;
  
  // Get inputs
  geometry_msgs::msg::Pose start_pose;
  if (!getInput("start_pose", start_pose))
  {
      start_pose.orientation.w = 1.0; 
  }
  
  double square_size = 10.0;
  getInput("square_size", square_size);
  
  double convergence_threshold = 0.5;
  getInput("convergence_threshold", convergence_threshold);
  
  std::string service_name;
  if (getInput("service_name", service_name) && service_name != client_->get_service_name())
  {
      client_ = node_->create_client<vortex_msgs::srv::SendWaypoints>(service_name);
  }

  // Generate waypoints
  auto waypoint_poses = generateSquareWaypoints(start_pose, square_size);
  
  // Prepare service request
  auto request = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
  for (const auto& pose : waypoint_poses)
  {
    vortex_msgs::msg::Waypoint wp;
    wp.pose = pose;
    wp.mode = vortex_msgs::msg::Waypoint::FULL_POSE;
    request->waypoints.push_back(wp);
  }
  request->switching_threshold = convergence_threshold;
  request->overwrite_prior_waypoints = true; 
  request->take_priority = true;

  // Call service asynchronously
  if (!client_->wait_for_service(std::chrono::seconds(1)))
  {
      RCLCPP_ERROR(node_->get_logger(), "Service %s not available", client_->get_service_name());
      return BT::NodeStatus::FAILURE;
  }

  // Send async request and store future
  future_ = client_->async_send_request(request).future.share();
  RCLCPP_INFO(node_->get_logger(), "SearchPattern: Sending waypoints...");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchPattern::onRunning()
{
  // If waypoints haven't been confirmed sent yet
  if (!waypoints_sent_)
  {
    if (future_.valid() && 
        future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      try
      {
         auto result = future_.get();
         if (!result->success)
         {
             RCLCPP_ERROR(node_->get_logger(), "WaypointAddition service returned false");
             return BT::NodeStatus::FAILURE;
         }
         
         waypoints_sent_ = true;
         start_time_ = std::chrono::system_clock::now();
         
         double duration = 60.0;
         getInput("duration", duration);
         RCLCPP_INFO(node_->get_logger(), "SearchPattern: Waypoints sent. Waiting for %.1f seconds...", duration);
      }
      catch (const std::exception& e)
      {
         RCLCPP_ERROR(node_->get_logger(), "SearchPattern: Exception calling service: %s", e.what());
         return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  // Waypoints sent, check duration
  double duration = 60.0;
  getInput("duration", duration);

  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now() - start_time_).count();

  if (elapsed >= duration)
  {
    RCLCPP_INFO(node_->get_logger(), "SearchPattern: Completed.");
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

}  // namespace mission_execution

// Plugin registration is done in plugin_register.cpp


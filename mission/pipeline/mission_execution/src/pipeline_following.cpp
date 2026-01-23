#include "mission_execution/pipeline_following.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace mission_execution
{

PipelineFollowing::PipelineFollowing(const std::string& name, const BT::NodeConfig& conf,
                                     const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, conf), node_(params.nh)
{
  client_ = node_->create_client<vortex_msgs::srv::SendWaypoints>("/orca/waypoint_addition");
}

BT::NodeStatus PipelineFollowing::onStart()
{
  waypoints_sent_ = false;

  // Get inputs
  geometry_msgs::msg::Pose start_pose;
  if (!getInput("start_pose", start_pose))
  {
      start_pose.orientation.w = 1.0; 
  }
  
  double convergence_threshold = 0.5;
  getInput("convergence_threshold", convergence_threshold);

  std::string service_name;
  if (getInput("service_name", service_name) && service_name != client_->get_service_name())
  {
      client_ = node_->create_client<vortex_msgs::srv::SendWaypoints>(service_name);
  }

  // Generate next waypoint (simple forward step)
  geometry_msgs::msg::Pose next_pose = start_pose;
  next_pose.position.x += 2.0;

  // Prepare service request
  auto request = std::make_shared<vortex_msgs::srv::SendWaypoints::Request>();
  vortex_msgs::msg::Waypoint wp;
  wp.pose = next_pose;
  wp.mode = vortex_msgs::msg::Waypoint::FULL_POSE;
  request->waypoints.push_back(wp);
  
  request->switching_threshold = convergence_threshold;
  request->overwrite_prior_waypoints = true;
  request->take_priority = true;

  // Output the final pose (which becomes the start pose for the next iteration)
  setOutput("final_pose", next_pose);

  // Call service asynchronously
  if (!client_->wait_for_service(std::chrono::seconds(1)))
  {
      RCLCPP_ERROR(node_->get_logger(), "Service %s not available", client_->get_service_name());
      return BT::NodeStatus::FAILURE;
  }

  // Send async request and store future
  future_ = client_->async_send_request(request).future.share();
  RCLCPP_INFO(node_->get_logger(), "PipelineFollowing: Sending waypoints...");
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PipelineFollowing::onRunning()
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
         RCLCPP_INFO(node_->get_logger(), "PipelineFollowing: Waypoint sent. Waiting for %.1f seconds...", duration);
      }
      catch (const std::exception& e)
      {
         RCLCPP_ERROR(node_->get_logger(), "PipelineFollowing: Exception calling service: %s", e.what());
         return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  double duration = 60.0;
  getInput("duration", duration);

  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now() - start_time_).count();

  if (elapsed >= duration)
  {
    RCLCPP_INFO(node_->get_logger(), "PipelineFollowing: Segment completed.");
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}

}  // namespace mission_execution

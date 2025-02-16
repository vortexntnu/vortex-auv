#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <line_filtering/line_filtering_visualization.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

// This function creates a SceneUpdate message visualizing the new tracks.
// It publishes for each track:
//   - A cone primitive at the midpoint (representing the angle gate)
//   - A line primitive (perpendicular to the track) representing the length gate.
foxglove_msgs::msg::SceneUpdate visualize_track_gates(
    const std::vector<Track>& tracks,
    const rclcpp::Time & timestamp,
    const std::string & frame_id,
    double gate_threshold,
    double gate_min_threshold,
    double gate_max_threshold)
  {
    // Create a scene entity message
    foxglove_msgs::msg::SceneEntity scene_entity;
    scene_entity.timestamp = timestamp; // Set timestamp
    scene_entity.id = 1; // Set entity ID
    scene_entity.frame_id = frame_id; // Set entity frame ID
    scene_entity.lifetime.sec = 5; // Set entity lifetime
    scene_entity.lifetime.nanosec = 0;
    scene_entity.frame_locked = false; // Set entity frame locked

    foxglove_msgs::msg::CylinderPrimitive cylinder;
  
    for (const auto & track : tracks) {
  
      // Compute the midpoint of the track segment.
      Eigen::Vector2d position = track.state.mean();
  
      Eigen::Matrix2d position_covariance = track.state.cov();

      vortex::prob::Gauss2d gauss(position, position_covariance);

      vortex::utils::Ellipse cov_ellipse = vortex::plotting::gauss_to_ellipse(gauss, gate_threshold);

      double major_axis = cov_ellipse.major_axis();
      major_axis = (major_axis < gate_min_threshold) ? gate_min_threshold : major_axis;
      major_axis = (major_axis > gate_max_threshold) ? gate_max_threshold : major_axis;

      double minor_axis = cov_ellipse.minor_axis();
      minor_axis = (minor_axis < gate_min_threshold) ? gate_min_threshold : minor_axis;
      minor_axis = (minor_axis > gate_max_threshold) ? gate_max_threshold : minor_axis;

      // Create a cylinder primitive
      cylinder.pose.position.x = position(0);
      cylinder.pose.position.y = position(1);
      cylinder.pose.position.z = 0.5;
      cylinder.pose.orientation.x = 0.0;
      cylinder.pose.orientation.y = 0.0;
      cylinder.pose.orientation.z = 0.0;
      cylinder.pose.orientation.w = 1.0;
      cylinder.size.x = major_axis;
      cylinder.size.y = minor_axis;
      cylinder.size.z = 2.0; 
      cylinder.bottom_scale = 1.0; 
      cylinder.top_scale = 1.0; 
      cylinder.color.r = 1.0;
      cylinder.color.g = 0.0;
      cylinder.color.b = 0.0;
      cylinder.color.a = 0.3;

      scene_entity.cylinders.push_back(cylinder);
  
    }

    foxglove_msgs::msg::SceneUpdate update;
    update.entities.push_back(scene_entity);
  
    return update;
  }
  

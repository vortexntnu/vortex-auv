#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <line_filtering/line_filtering_visualization.hpp>

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
    foxglove_msgs::msg::SceneUpdate scene_update;
  
    for (const auto & track : tracks) {
      // Retrieve endpoints (assuming each column is one endpoint)
      Eigen::Vector2d p0 = track.line_points.col(0);
      Eigen::Vector2d p1 = track.line_points.col(1);
  
      // Compute the midpoint of the track segment.
      Eigen::Vector2d midpoint = (p0 + p1) / 2.0;
  
      // Compute the direction along the track (from p0 to p1) and normalize.
      Eigen::Vector2d direction = (p1 - p0);
      double norm = direction.norm();
      if (norm > 0)
        direction /= norm;
  
      // Compute a normal (perpendicular) vector to the track (if needed elsewhere).
      Eigen::Vector2d normal(-direction.y(), direction.x());
  
      // ----- Create Two Line Primitives for the Angle Gate -----
      // Define a fixed angular aperture for the gate (e.g., 30°)
      double angle_aperture_deg = 30.0;
      double angle_aperture_rad = angle_aperture_deg * M_PI / 180.0;
      double half_aperture = angle_aperture_rad / 2.0;
  
      // Determine the track's current angle (in radians)
      double track_angle = std::atan2(direction.y(), direction.x());
  
      // Compute the boundary angles for the gate.
      double boundary_angle1 = track_angle + half_aperture;
      double boundary_angle2 = track_angle - half_aperture;
  
      // Determine endpoints for the angle gate boundaries. We use gate_threshold as the length.
      Eigen::Vector2d boundary_endpoint1 = midpoint + gate_threshold * Eigen::Vector2d(std::cos(boundary_angle1), std::sin(boundary_angle1));
      Eigen::Vector2d boundary_endpoint2 = midpoint + gate_threshold * Eigen::Vector2d(std::cos(boundary_angle2), std::sin(boundary_angle2));
  
      // Create the first angle boundary line.
      foxglove_msgs::msg::LinePrimitive angle_line1;
      angle_line1.type = 0;  // Use appropriate type for a basic line segment.
      angle_line1.thickness = 0.1;
      angle_line1.scale_invariant = false;
      // Set the color (blue, fully opaque).
      angle_line1.color.r = 0.0;
      angle_line1.color.g = 0.0;
      angle_line1.color.b = 1.0;
      angle_line1.color.a = 1.0;
      // Define the start and end points (with a fixed z offset).
      geometry_msgs::msg::Point start_pt;
      start_pt.x = midpoint.x();
      start_pt.y = midpoint.y();
      start_pt.z = 0.5;
      geometry_msgs::msg::Point end_pt1;
      end_pt1.x = boundary_endpoint1.x();
      end_pt1.y = boundary_endpoint1.y();
      end_pt1.z = 0.5;
      angle_line1.points.push_back(start_pt);
      angle_line1.points.push_back(end_pt1);
  
      // Create the second angle boundary line.
      foxglove_msgs::msg::LinePrimitive angle_line2;
      angle_line2.type = 0;
      angle_line2.thickness = 0.1;
      angle_line2.scale_invariant = false;
      // Set the color (blue, fully opaque).
      angle_line2.color.r = 0.0;
      angle_line2.color.g = 0.0;
      angle_line2.color.b = 1.0;
      angle_line2.color.a = 1.0;
      geometry_msgs::msg::Point end_pt2;
      end_pt2.x = boundary_endpoint2.x();
      end_pt2.y = boundary_endpoint2.y();
      end_pt2.z = 0.5;
      angle_line2.points.push_back(start_pt);
      angle_line2.points.push_back(end_pt2);
  
      // ----- Create the Line Primitive for the Length Gate -----
      // Use gate_threshold as the base value, clamped between min and max.
      double length_gate = gate_threshold;
      if (length_gate < gate_min_threshold) {
        length_gate = gate_min_threshold;
      }
      if (length_gate > gate_max_threshold) {
        length_gate = gate_max_threshold;
      }
      // Offset along the normal: half the length in each direction.
      Eigen::Vector2d offset = (length_gate / 2.0) * normal;
      geometry_msgs::msg::Point pointA;
      pointA.x = midpoint.x() + offset.x();
      pointA.y = midpoint.y() + offset.y();
      pointA.z = 0.5;  // same z offset as above
      geometry_msgs::msg::Point pointB;
      pointB.x = midpoint.x() - offset.x();
      pointB.y = midpoint.y() - offset.y();
      pointB.z = 0.5;
  
      foxglove_msgs::msg::LinePrimitive length_line;
      length_line.type = 0;
      length_line.thickness = 0.1;
      length_line.scale_invariant = false;
      // Set the color (red, fully opaque).
      length_line.color.r = 1.0;
      length_line.color.g = 0.0;
      length_line.color.b = 0.0;
      length_line.color.a = 1.0;
      length_line.points.push_back(pointA);
      length_line.points.push_back(pointB);
  
      // ----- Bundle Primitives into a Scene Entity -----
      foxglove_msgs::msg::SceneEntity entity;
      entity.timestamp = timestamp;
      entity.id = track.id;
      entity.frame_id = frame_id;
      // Display the entity for 5 seconds.
      entity.lifetime.sec = 5;
      entity.lifetime.nanosec = 0;
      entity.frame_locked = false;
  
      // Add the angle gate boundary lines instead of a cone.
      entity.lines.push_back(angle_line1);
      entity.lines.push_back(angle_line2);
      // Also include the length gate line.
      entity.lines.push_back(length_line);
  
      // Add this entity to the scene update message.
      scene_update.entities.push_back(entity);
    }
  
    return scene_update;
  }
  

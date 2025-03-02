#include <line_filtering/line_filtering_visualization.hpp>

foxglove_msgs::msg::SceneUpdate visualize_track_gates(
    const std::vector<Track>& tracks,
    const rclcpp::Time & timestamp,
    const std::string & frame_id,
    double gate_threshold,
    double gate_min_threshold,
    double gate_max_threshold,
    bool red,
    double orca_depth,
    double dvl_altitude)
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
      cylinder.pose.position.z = orca_depth + dvl_altitude;
      cylinder.pose.orientation.x = 0.0;
      cylinder.pose.orientation.y = 0.0;
      cylinder.pose.orientation.z = 0.0;
      cylinder.pose.orientation.w = 1.0;
      cylinder.size.x = major_axis;
      cylinder.size.y = minor_axis;
      cylinder.size.z = 2.0; 
      cylinder.bottom_scale = 1.0; 
      cylinder.top_scale = 1.0;
      if (red) {
        cylinder.color.r = 1.0;
        cylinder.color.g = 0.0;
        cylinder.color.b = 0.0;
        cylinder.color.a = 0.3;
      } else {
        cylinder.color.r = 0.0;
        cylinder.color.g = 1.0;
        cylinder.color.b = 0.0;
        cylinder.color.a = 0.3;
      }

      scene_entity.cylinders.push_back(cylinder);
  
    }

    foxglove_msgs::msg::SceneUpdate update;
    update.entities.push_back(scene_entity);
  
    return update;
  }

  visualization_msgs::msg::MarkerArray visualize_line_tracks(const std::vector<Track>& tracks,
    const rclcpp::Time & timestamp,
    const std::string & frame_id,
    double orca_depth,
    double dvl_altitude) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;
    marker.ns = "track_points";
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    for (const auto& track : tracks) {
        if (!track.confirmed) {
            continue;
        }

        geometry_msgs::msg::Point start;
        start.x = track.line_points(0, 0);
        start.y = track.line_points(1, 0);
        start.z = orca_depth + dvl_altitude;

        geometry_msgs::msg::Point end;
        end.x = track.line_points(0, 1);
        end.y = track.line_points(1, 1);
        end.z = orca_depth + dvl_altitude;

        marker.points.push_back(start);
        marker.points.push_back(end);
    }

    marker_array.markers.push_back(marker);

    return marker_array;

}
  

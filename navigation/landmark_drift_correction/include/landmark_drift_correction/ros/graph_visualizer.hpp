#ifndef LANDMARK_DRIFT_CORRECTION__ROS__GRAPH_VISUALIZER_HPP_
#define LANDMARK_DRIFT_CORRECTION__ROS__GRAPH_VISUALIZER_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <landmark_drift_correction/lib/drift_corrector.hpp>

namespace vortex::navigation::drift_correction {

/// Builds a MarkerArray visualizing the full keyframe graph:
///   - SPHERE_LIST  : one sphere per node
///   - LINE_STRIP   : edges connecting consecutive nodes
visualization_msgs::msg::MarkerArray build_graph_markers(
    const std::vector<Keyframe>& keyframes,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

}  // namespace vortex::navigation::drift_correction

#endif  // LANDMARK_DRIFT_CORRECTION__ROS__GRAPH_VISUALIZER_HPP_

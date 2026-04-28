#include "landmark_drift_correction/ros/graph_visualizer.hpp"

#include <geometry_msgs/msg/point.hpp>

namespace vortex::navigation {

visualization_msgs::msg::MarkerArray build_graph_markers(
    const std::vector<Keyframe>& keyframes,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray array;

    if (keyframes.empty()) {
        return array;
    }

    // --- Node spheres ---
    {
        visualization_msgs::msg::Marker nodes;
        nodes.header.frame_id = frame_id;
        nodes.header.stamp = stamp;
        nodes.ns = "keyframe_nodes";
        nodes.id = 0;
        nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::msg::Marker::ADD;
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.05;
        nodes.color.r = 1.0f;
        nodes.color.g = 0.1f;
        nodes.color.b = 0.1f;
        nodes.color.a = 1.0f;

        for (const auto& kf : keyframes) {
            geometry_msgs::msg::Point p;
            p.x = kf.translation.x();
            p.y = kf.translation.y();
            p.z = kf.translation.z();
            nodes.points.push_back(p);
        }

        array.markers.push_back(nodes);
    }

    // --- Edges as LINE_STRIP between consecutive nodes ---
    if (keyframes.size() >= 2) {
        visualization_msgs::msg::Marker edges;
        edges.header.frame_id = frame_id;
        edges.header.stamp = stamp;
        edges.ns = "keyframe_edges";
        edges.id = 1;
        edges.type = visualization_msgs::msg::Marker::LINE_STRIP;
        edges.action = visualization_msgs::msg::Marker::ADD;
        edges.scale.x = 0.025;
        edges.color.r = 1.0f;
        edges.color.g = 1.0f;
        edges.color.b = 0.1f;
        edges.color.a = 0.8f;

        for (const auto& kf : keyframes) {
            geometry_msgs::msg::Point p;
            p.x = kf.translation.x();
            p.y = kf.translation.y();
            p.z = kf.translation.z();
            edges.points.push_back(p);
        }

        array.markers.push_back(edges);
    }

    return array;
}

}  // namespace vortex::navigation

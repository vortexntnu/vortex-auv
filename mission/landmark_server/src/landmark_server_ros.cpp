#include "landmark_server/landmark_server_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace vortex::mission {

LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landmark_server_node", options) {}

}  // namespace vortex::mission

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::mission::LandmarkServerNode)

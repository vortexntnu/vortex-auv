#ifndef LANDMARK_SERVER__ROS_CONFIG_HPP_
#define LANDMARK_SERVER__ROS_CONFIG_HPP_

#include <rclcpp/rclcpp.hpp>

#include "landmark_server/config.hpp"

namespace vortex::landmark_server {

/// From the node's parameters. Throws on a bad value.
Config load_config(rclcpp::Node& node);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__ROS_CONFIG_HPP_

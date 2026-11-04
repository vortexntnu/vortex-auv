#ifndef LANDMARK_SERVER__MARKERS_HPP_
#define LANDMARK_SERVER__MARKERS_HPP_

#include <cstdint>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vortex_msgs/msg/landmark_track.hpp>

#include "landmark_server/landmark.hpp"

namespace vortex::landmark_server {

std_msgs::msg::ColorRGBA type_color(std::uint16_t type);

/// A cube or real-size box, a label, and an arrow along +X when the yaw is
/// known. Grey when not seen lately.
void add_markers(const Landmark& l,
                 const vortex_msgs::msg::LandmarkTrack& track,
                 visualization_msgs::msg::MarkerArray& out);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__MARKERS_HPP_

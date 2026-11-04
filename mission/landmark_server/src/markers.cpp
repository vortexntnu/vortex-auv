#include "landmark_server/markers.hpp"

#include <fmt/format.h>

#include <array>

#include <vortex_msgs/msg/landmark_type.hpp>

#include "landmark_server/geometry.hpp"

namespace vortex::landmark_server {

std_msgs::msg::ColorRGBA type_color(std::uint16_t type) {
    using LT = vortex_msgs::msg::LandmarkType;
    std::array<float, 3> c{0.7F, 0.7F, 0.7F};
    switch (type) {
        case LT::GATE:
            c = {1.0F, 0.6F, 0.0F};
            break;
        case LT::SLALOM_PIPE:
            c = {0.9F, 0.2F, 0.2F};
            break;
        case LT::TORPEDO_BOARD:
            c = {0.2F, 0.6F, 1.0F};
            break;
        case LT::BIN:
            c = {0.3F, 0.8F, 0.3F};
            break;
        case LT::PATH_MARKER:
            c = {0.9F, 0.9F, 0.2F};
            break;
        case LT::TABLE:
            c = {0.6F, 0.4F, 0.2F};
            break;
        case LT::OCTAGON:
            c = {0.8F, 0.3F, 0.9F};
            break;
        default:
            break;
    }
    std_msgs::msg::ColorRGBA out;
    out.r = c[0];
    out.g = c[1];
    out.b = c[2];
    out.a = 1.0F;
    return out;
}

/// Per landmark: a cube or its real-size box, a label, and an arrow
/// along +X when the yaw is known. Grey when not seen lately.
void add_markers(const Landmark& l,
                 const vortex_msgs::msg::LandmarkTrack& track,
                 visualization_msgs::msg::MarkerArray& out) {
    using visualization_msgs::msg::Marker;
    const float alpha = 0.9F;
    const Eigen::Matrix3d R =
        yaw_pose(l.yaw_known ? l.yaw : 0.0, l.position).rotation();
    const Eigen::Quaterniond q(R);

    Marker m;
    m.header = track.header;
    m.id = l.id;
    m.pose.position = track.landmark.pose.pose.position;
    m.pose.orientation.w = 1.0;
    m.color = type_color(l.cls.type);
    if (track.retained) {
        m.color.r = m.color.g = m.color.b = 0.5F;
    }
    m.color.a = alpha;

    if (!l.cls.box || !l.cls.box->solid) {
        Marker point = m;
        point.ns = "landmark";
        point.type = Marker::CUBE;
        point.scale.x = point.scale.y = point.scale.z = 0.25;
        out.markers.push_back(point);
    }

    if (l.cls.box) {
        const MarkerBox& box = *l.cls.box;
        const Eigen::Vector3d centre =
            l.position +
            R * Eigen::Vector3d(box.offset[0], box.offset[1], box.offset[2]);
        Marker b = m;
        b.ns = "structure";
        b.type = Marker::CUBE;
        b.pose.position.x = centre.x();
        b.pose.position.y = centre.y();
        b.pose.position.z = centre.z();
        b.pose.orientation.w = q.w();
        b.pose.orientation.x = q.x();
        b.pose.orientation.y = q.y();
        b.pose.orientation.z = q.z();
        b.scale.x = box.size[0];
        b.scale.y = box.size[1];
        b.scale.z = box.size[2];
        if (box.color && !track.retained) {
            b.color.r = static_cast<float>((*box.color)[0]);
            b.color.g = static_cast<float>((*box.color)[1]);
            b.color.b = static_cast<float>((*box.color)[2]);
        }
        b.color.a = box.solid ? alpha : 0.25F;
        out.markers.push_back(b);
    }

    Marker label = m;
    label.ns = "label";
    label.type = Marker::TEXT_VIEW_FACING;
    label.pose.position.z -= 0.35;
    label.scale.z = 0.18;
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
    label.text = fmt::format("{} #{} (n {})", l.cls.name, l.id, l.n_obs);
    out.markers.push_back(label);

    if (l.yaw_known) {
        Marker arrow = m;
        arrow.ns = "front";
        arrow.type = Marker::ARROW;
        arrow.pose.orientation.w = q.w();
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();
        arrow.scale.x = 0.8;
        arrow.scale.y = arrow.scale.z = 0.06;
        arrow.color.a = 1.0F;
        out.markers.push_back(arrow);
    }
}

}  // namespace vortex::landmark_server

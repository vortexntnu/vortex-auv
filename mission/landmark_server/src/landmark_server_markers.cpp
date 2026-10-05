#include <cstdio>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include "landmark_server/landmark_server_ros.hpp"

// The map view for Foxglove/RViz (landmark_server/markers, debug.markers):
// the landmarks, the course tasks and the course frame. Display only.

namespace vortex::mission {

namespace {

std_msgs::msg::ColorRGBA color_for(uint16_t type, float alpha) {
    // One colour per landmark type, so the map reads at a glance.
    struct Rgb {
        float r, g, b;
    };
    Rgb c{0.7F, 0.7F, 0.7F};
    switch (type) {
        case vortex_msgs::msg::LandmarkType::GATE:
            c = {1.0F, 0.6F, 0.0F};
            break;
        case vortex_msgs::msg::LandmarkType::SLALOM_PIPE:
            c = {0.9F, 0.2F, 0.2F};
            break;
        case vortex_msgs::msg::LandmarkType::TORPEDO_BOARD:
            c = {0.2F, 0.6F, 1.0F};
            break;
        case vortex_msgs::msg::LandmarkType::BIN:
            c = {0.3F, 0.8F, 0.3F};
            break;
        case vortex_msgs::msg::LandmarkType::PATH_MARKER:
            c = {0.9F, 0.9F, 0.2F};
            break;
        case vortex_msgs::msg::LandmarkType::TABLE:
            c = {0.6F, 0.4F, 0.2F};
            break;
        case vortex_msgs::msg::LandmarkType::OCTAGON:
            c = {0.8F, 0.3F, 0.9F};
            break;
        default:
            break;
    }
    std_msgs::msg::ColorRGBA out;
    out.r = c.r;
    out.g = c.g;
    out.b = c.b;
    out.a = alpha;
    return out;
}

/// Torpedo board parts: the openings (TORPEDO_TARGET_*) as yellow spheres,
/// large or small, and the icons as flat magenta squares on the board face,
/// so the two are not mixed up. Other landmarks are left as they are.
void style_torpedo_part(const RetainedLandmark& lm,
                        float alpha,
                        const RetainedLandmark* board,
                        visualization_msgs::msg::Marker& m) {
    using visualization_msgs::msg::Marker;
    using LS = vortex_msgs::msg::LandmarkSubtype;
    if (lm.key.type != vortex_msgs::msg::LandmarkType::TORPEDO_BOARD) {
        return;
    }
    switch (lm.key.subtype) {
        case LS::TORPEDO_TARGET_LARGE_SEARCH_RESCUE:
        case LS::TORPEDO_TARGET_LARGE_SURVEY_REPAIR:
        case LS::TORPEDO_TARGET_SMALL_SEARCH_RESCUE:
        case LS::TORPEDO_TARGET_SMALL_SURVEY_REPAIR: {
            const bool large =
                lm.key.subtype == LS::TORPEDO_TARGET_LARGE_SEARCH_RESCUE ||
                lm.key.subtype == LS::TORPEDO_TARGET_LARGE_SURVEY_REPAIR;
            m.type = Marker::SPHERE;
            m.scale.x = m.scale.y = m.scale.z = large ? 0.25 : 0.18;
            m.color.r = 1.0F;
            m.color.g = 0.85F;
            m.color.b = 0.0F;
            m.color.a = alpha;
            break;
        }
        case LS::TORPEDO_ICON_FIRE:
        case LS::TORPEDO_ICON_BLOOD:
        case LS::TORPEDO_ICON_FIRETRUCK:
        case LS::TORPEDO_ICON_AMBULANCE:
            // Thin along +X (out of the board face), turned with the board
            // when its yaw is known.
            m.type = Marker::CUBE;
            m.scale.x = 0.04;
            m.scale.y = m.scale.z = 0.2;
            if (board != nullptr) {
                m.pose.orientation.w = board->orientation.w();
                m.pose.orientation.x = board->orientation.x();
                m.pose.orientation.y = board->orientation.y();
                m.pose.orientation.z = board->orientation.z();
            }
            m.color.r = 1.0F;
            m.color.g = 0.2F;
            m.color.b = 0.8F;
            m.color.a = alpha;
            break;
        default:
            break;
    }
}

}  // namespace

void LandmarkServerNode::publish_markers() {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    // The map is republished whole every tick.
    Marker clear;
    clear.header.frame_id = target_frame_;
    clear.header.stamp = stamp;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    const auto base = [&](const RetainedLandmark& lm, const char* ns) {
        Marker m;
        m.header.frame_id = target_frame_;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = lm.id;
        m.action = Marker::ADD;
        m.pose.position.x = lm.position.x();
        m.pose.position.y = lm.position.y();
        m.pose.position.z = lm.position.z();
        m.pose.orientation.w = 1.0;
        return m;
    };

    // The torpedo board with a known yaw: the icon squares lie on its face.
    const RetainedLandmark* board = nullptr;
    for (const auto& lm : map_->landmarks()) {
        if (lm.absorbed_by < 0 && lm.has_orientation &&
            lm.key.type == vortex_msgs::msg::LandmarkType::TORPEDO_BOARD &&
            lm.key.subtype ==
                vortex_msgs::msg::LandmarkSubtype::TORPEDO_BOARD_WHOLE) {
            board = &lm;
            break;
        }
    }

    for (const auto& lm : map_->landmarks()) {
        if (lm.absorbed_by >= 0) {
            continue;
        }
        // Remembered landmarks (not seen now) are faded.
        const float alpha = lm.is_live() ? 1.0F : 0.55F;
        const auto color = color_for(lm.key.type, alpha);

        const auto* box = map_config_.marker_box_for(lm.key);

        // The point. A class drawn as a solid object below (a thin pipe)
        // also gets a sphere, so it is easy to spot. The whole gate has its
        // outline box, arrow and label; a cube in its middle would look like
        // a third poster plate.
        const bool gate_whole =
            lm.key.type == vortex_msgs::msg::LandmarkType::GATE &&
            lm.key.subtype == vortex_msgs::msg::LandmarkSubtype::GATE_WHOLE;
        if (!gate_whole) {
            const bool solid = box != nullptr && box->solid;
            Marker point = base(lm, "landmark");
            point.type = solid ? Marker::SPHERE : Marker::CUBE;
            point.scale.x = point.scale.y = point.scale.z = solid ? 0.15 : 0.3;
            point.color = color;
            if (solid && box->color) {
                point.color.r = static_cast<float>(box->color->x());
                point.color.g = static_cast<float>(box->color->y());
                point.color.b = static_cast<float>(box->color->z());
            }
            style_torpedo_part(lm, alpha, board, point);
            array.markers.push_back(point);
        }

        // The name, with id and age since the last measurement.
        Marker label = base(lm, "label");
        label.type = Marker::TEXT_VIEW_FACING;
        label.pose.position.z -= 0.35;
        label.scale.z = 0.18;
        label.color.r = label.color.g = label.color.b = 1.0F;
        label.color.a = 1.0F;
        char age[32];
        std::snprintf(age, sizeof(age), "%.1f",
                      stamp.seconds() - lm.last_measurement);
        label.text = class_name(lm.key) + " #" + std::to_string(lm.id) + " (" +
                     age + " s)" + (lm.derived ? " derived" : "");
        array.markers.push_back(label);

        // The real size: a solid PVC pipe, or a see-through box around a
        // large structure. Turned with the yaw when it is known (else along
        // the odom axes).
        if (box != nullptr) {
            Marker m = base(lm, "structure");
            m.type = Marker::CUBE;
            const Eigen::Quaterniond q = lm.has_orientation
                                             ? lm.orientation
                                             : Eigen::Quaterniond::Identity();
            const Eigen::Vector3d centre = lm.position + q * box->offset;
            m.pose.position.x = centre.x();
            m.pose.position.y = centre.y();
            m.pose.position.z = centre.z();
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();
            m.pose.orientation.w = q.w();
            m.scale.x = box->size.x();
            m.scale.y = box->size.y();
            m.scale.z = box->size.z();
            m.color = color;
            if (box->color) {
                m.color.r = static_cast<float>(box->color->x());
                m.color.g = static_cast<float>(box->color->y());
                m.color.b = static_cast<float>(box->color->z());
            }
            if (box->solid) {
                m.color.a = alpha;
            } else {
                m.color.a = lm.is_live() ? 0.15F : 0.06F;
            }
            array.markers.push_back(m);
        }

        // +X out of the front, when the orientation is known.
        if (lm.has_orientation) {
            Marker arrow = base(lm, "front");
            arrow.type = Marker::ARROW;
            arrow.pose.orientation.x = lm.orientation.x();
            arrow.pose.orientation.y = lm.orientation.y();
            arrow.pose.orientation.z = lm.orientation.z();
            arrow.pose.orientation.w = lm.orientation.w();
            arrow.scale.x = 0.8;
            arrow.scale.y = 0.06;
            arrow.scale.z = 0.06;
            arrow.color = color;
            arrow.color.a = 1.0F;
            array.markers.push_back(arrow);
        }
    }

    // Course tasks: a placed task as lines from its origin to every part
    // and the parts never seen as spheres where the template expects them;
    // a task not placed yet as the circle it is searched in. Name, parts
    // seen, variant, locked / committed.
    {
        const CourseModel& course = map_->course();
        const auto point = [](const Eigen::Vector3d& v) {
            geometry_msgs::msg::Point p;
            p.x = v.x();
            p.y = v.y();
            p.z = v.z();
            return p;
        };
        int task_id = 0;
        for (const auto& t : course.tasks()) {
            if (!t.placed && course_->status() == CourseFrameStatus::UNSET) {
                continue;
            }
            const bool locked = course.locked(t);
            visualization_msgs::msg::Marker lines;
            lines.header.frame_id = target_frame_;
            lines.header.stamp = stamp;
            lines.ns = "course_task";
            lines.id = task_id;
            lines.action = visualization_msgs::msg::Marker::ADD;
            lines.type = visualization_msgs::msg::Marker::LINE_LIST;
            lines.pose.orientation.w = 1.0;
            lines.scale.x = 0.03;
            lines.color.r = locked ? 0.5F : 0.2F;
            lines.color.g = locked ? 0.5F : 0.9F;
            lines.color.b = locked ? 0.5F : 0.9F;
            lines.color.a = t.placed ? 0.9F : 0.4F;
            auto open = lines;
            open.ns = "course_open_parts";
            open.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            open.scale.x = open.scale.y = open.scale.z = 0.15;
            open.color.a = 0.35F;
            int seen = 0;
            if (t.placed) {
                for (std::size_t i = 0; i < t.slots.size(); ++i) {
                    const Eigen::Vector3d p = course.slot_position(t, i);
                    lines.points.push_back(point(t.pose.translation()));
                    lines.points.push_back(point(p));
                    if (t.slots[i].landmark_id < 0) {
                        open.points.push_back(point(p));
                    } else {
                        ++seen;
                    }
                }
            } else {
                // The search region: a circle around the prior.
                lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
                const double r = t.spec->region_radius_m;
                for (int k = 0; k <= 36; ++k) {
                    const double a = 2.0 * M_PI * k / 36.0;
                    lines.points.push_back(point(
                        t.pose.translation() +
                        Eigen::Vector3d(r * std::cos(a), r * std::sin(a), 0.0)));
                }
            }
            array.markers.push_back(lines);
            if (!open.points.empty()) {
                array.markers.push_back(open);
            }
            auto label = lines;
            label.ns = "course_task_label";
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.points.clear();
            label.pose.position = point(t.pose.translation() -
                                        Eigen::Vector3d(0.0, 0.0, 0.7));
            label.scale.z = 0.2;
            label.color.a = 1.0F;
            std::string text = t.spec->name;
            if (t.placed) {
                text += " (" + std::to_string(seen) + "/" +
                        std::to_string(t.slots.size()) + ")";
                const auto v = course.variant_name(t);
                if (t.tmpl->variants.size() > 1) {
                    text += v.empty() ? " ?" : " " + v;
                }
            } else {
                text += " (searching)";
            }
            if (locked) {
                text += " locked";
            }
            if (t.committed) {
                text += " committed";
            }
            label.text = text;
            array.markers.push_back(label);
            ++task_id;
        }
    }

    // The course frame: its origin, the direction through the gate and the
    // lane (the area the course layout covers, aligned to the tasks found).
    if (course_->status() != CourseFrameStatus::UNSET) {
        const auto point3 = [&](const Eigen::Vector2d& odom_xy) {
            geometry_msgs::msg::Point p;
            p.x = odom_xy.x();
            p.y = odom_xy.y();
            p.z = 0.0;
            return p;
        };
        const bool locked = course_->status() == CourseFrameStatus::GATE_LOCKED;

        Marker lane;
        lane.header.frame_id = target_frame_;
        lane.header.stamp = stamp;
        lane.ns = "course_lane";
        lane.id = 0;
        lane.action = Marker::ADD;
        lane.type = Marker::LINE_STRIP;
        lane.pose.orientation.w = 1.0;
        lane.scale.x = 0.05;
        lane.color.r = locked ? 0.2F : 0.9F;
        lane.color.g = locked ? 0.9F : 0.9F;
        lane.color.b = 0.2F;
        lane.color.a = 0.8F;
        const auto layout_lane = map_->course().lane_corners(course_geometry());
        for (std::size_t k = 0; !layout_lane.empty() && k <= layout_lane.size(); ++k) {
            lane.points.push_back(point3(layout_lane[k % layout_lane.size()]));
        }
        if (!lane.points.empty()) {
            array.markers.push_back(lane);
        }

        Marker axis;
        axis.header = lane.header;
        axis.ns = "course_axis";
        axis.id = 0;
        axis.action = Marker::ADD;
        axis.type = Marker::ARROW;
        axis.pose.orientation.w = 1.0;
        axis.scale.x = 0.08;
        axis.scale.y = 0.2;
        axis.scale.z = 0.2;
        axis.color = lane.color;
        axis.color.a = 1.0F;
        axis.points.push_back(point3(course_->from_course({0.0, 0.0})));
        axis.points.push_back(point3(course_->from_course({3.0, 0.0})));
        array.markers.push_back(axis);
    }

    markers_pub_->publish(array);
}

}  // namespace vortex::mission

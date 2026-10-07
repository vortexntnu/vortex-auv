#include "landmark_server/map_rules.hpp"
#include <algorithm>
#include <cmath>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::mission {

namespace {

using LT = vortex_msgs::msg::LandmarkType;
using LS = vortex_msgs::msg::LandmarkSubtype;
using vortex::filtering::LandmarkClassKey;

/// The first non-derived landmark of a class, if any.
RetainedLandmark* find_measured(RetainedLandmarks& map,
                                const LandmarkClassKey& key) {
    for (auto& lm : map.landmarks()) {
        if (!lm.derived && lm.key == key) {
            return &lm;
        }
    }
    return nullptr;
}

bool is_fresh(const RetainedLandmark& lm) {
    return lm.live_track_id >= 0;
}

/// Unit vector perpendicular to a -> b in the horizontal plane, pointing
/// towards @p reference (a direction).
Eigen::Vector2d normal_towards(const Eigen::Vector2d& a,
                               const Eigen::Vector2d& b,
                               const Eigen::Vector2d& reference) {
    const Eigen::Vector2d d = (b - a).normalized();
    Eigen::Vector2d n(-d.y(), d.x());
    if (n.dot(reference) < 0.0) {
        n = -n;
    }
    return n;
}

double yaw_of_direction(const Eigen::Vector2d& d) {
    return std::atan2(d.y(), d.x());
}

/// A derived landmark hidden behind a measured one of the same class.
void absorb_derived_into_measured(RetainedLandmarks& map,
                                  const std::string& slot,
                                  const RetainedLandmark* measured) {
    for (auto& lm : map.landmarks()) {
        if (lm.derived && lm.derived_slot == slot) {
            lm.absorbed_by = measured != nullptr ? measured->id : -1;
        }
    }
}

// --- Gate -------------------------------------------------------------------

/// The course frame's gate estimate from the two panels, taken only while
/// both are being seen (the gate task's pose is refitted from remembered
/// parts every tick and would look consistent even when it is wrong).
void feed_course_frame_from_panels(RetainedLandmarks& map,
                                   const MapRulesConfig& cfg,
                                   const Eigen::Vector3d& vehicle,
                                   CourseFrameTracker* course) {
    if (course == nullptr) {
        return;
    }
    RetainedLandmark* survey =
        find_measured(map, {LT::GATE, LS::GATE_SURVEY_REPAIR});
    RetainedLandmark* rescue =
        find_measured(map, {LT::GATE, LS::GATE_SEARCH_RESCUE});
    if (survey == nullptr || rescue == nullptr || !is_fresh(*survey) ||
        !is_fresh(*rescue)) {
        return;
    }
    const Eigen::Vector2d a = survey->position.head<2>();
    const Eigen::Vector2d b = rescue->position.head<2>();
    const double separation = (b - a).norm();
    if (separation < cfg.min_panel_separation_m ||
        (cfg.max_panel_separation_m > 0.0 &&
         separation > cfg.max_panel_separation_m)) {
        return;
    }
    const Eigen::Vector3d midpoint =
        0.5 * (survey->position + rescue->position);
    // The normal towards the vehicle: the course frame takes the side the
    // gate was first seen from as its front (the course direction, through
    // the gate, points away from it).
    const Eigen::Vector2d reference = (vehicle - midpoint).head<2>();
    course->add_gate_estimate(
        midpoint.head<2>(), yaw_of_direction(normal_towards(a, b, reference)));
}

// --- Bins -------------------------------------------------------------------

void apply_bin_rules(RetainedLandmarks& map, const MapRulesConfig& cfg) {
    // Recompute from scratch every tick.
    for (auto& lm : map.landmarks()) {
        if (lm.key.type == LT::BIN && lm.key.subtype == LS::BIN_UNCLASSIFIED) {
            lm.absorbed_by = -1;
        }
    }
    if (!cfg.bin_role_from_down_icons) {
        return;
    }
    for (auto& role_bin : map.landmarks()) {
        if (role_bin.derived || role_bin.key.type != LT::BIN ||
            (role_bin.key.subtype != LS::BIN_SURVEY_REPAIR &&
             role_bin.key.subtype != LS::BIN_SEARCH_RESCUE)) {
            continue;
        }
        // The nearest bin without a role is the same bin.
        RetainedLandmark* nearest = nullptr;
        double best = cfg.bin_role_radius_m;
        for (auto& bin : map.landmarks()) {
            if (bin.derived || bin.key.type != LT::BIN ||
                bin.key.subtype != LS::BIN_UNCLASSIFIED ||
                bin.absorbed_by >= 0) {
                continue;
            }
            const double d =
                (bin.position - role_bin.position).head<2>().norm();
            if (d <= best) {
                best = d;
                nearest = &bin;
            }
        }
        if (nearest != nullptr) {
            nearest->absorbed_by = role_bin.id;
        }
    }
}

// --- Octagon ----------------------------------------------------------------

void apply_octagon_rules(RetainedLandmarks& map,
                         const MapRulesConfig& cfg,
                         double now) {
    if (!cfg.octagon_from_table) {
        return;
    }
    // The octagon floats over the table: one xy for both. The table stands on
    // the floor, the octagon at the water surface.
    RetainedLandmark* table = find_measured(map, {LT::TABLE, LS::TABLE_WHOLE});
    RetainedLandmark* octagon =
        find_measured(map, {LT::OCTAGON, LS::OCTAGON_WHOLE});
    if (table == nullptr && octagon == nullptr) {
        return;
    }
    absorb_derived_into_measured(map, "octagon_whole", octagon);
    absorb_derived_into_measured(map, "table_whole", table);

    Eigen::Vector2d xy;
    if (table != nullptr && octagon != nullptr) {
        if (cfg.table_octagon_primary == "octagon") {
            xy = octagon->position.head<2>();
        } else if (cfg.table_octagon_primary == "midpoint") {
            xy = 0.5 * (table->position + octagon->position).head<2>();
        } else {
            xy = table->position.head<2>();
        }
    } else {
        xy = (table != nullptr ? table : octagon)->position.head<2>();
    }

    if (octagon == nullptr) {
        octagon = &map.upsert_derived("octagon_whole",
                                      {LT::OCTAGON, LS::OCTAGON_WHOLE}, now);
        octagon->last_measurement = table->last_measurement;
        octagon->derived_live = is_fresh(*table);
        octagon->position.z() = table->position.z();
    } else if (table == nullptr && cfg.z_lock.enable) {
        // Without the floor depth the table top cannot be placed: a table at
        // the octagon's depth would send the vehicle to the surface.
        table = &map.upsert_derived("table_whole", {LT::TABLE, LS::TABLE_WHOLE},
                                    now);
        table->last_measurement = octagon->last_measurement;
        table->derived_live = is_fresh(*octagon);
        table->position.z() = cfg.z_lock.floor_z - cfg.table_height_m;
    }

    octagon->position.head<2>() = xy;
    if (cfg.z_lock.enable) {
        octagon->position.z() = cfg.z_lock.surface_z;
    }
    if (table != nullptr) {
        table->position.head<2>() = xy;
    }
}

}  // namespace

void apply_map_rules(RetainedLandmarks& map,
                     const MapRulesConfig& config,
                     const Eigen::Vector3d& vehicle_position,
                     double now,
                     CourseFrameTracker* course) {
    // The gate and the torpedo board (pose, yaw, roles, openings) come from
    // their course templates.
    feed_course_frame_from_panels(map, config, vehicle_position, course);
    apply_bin_rules(map, config);
    apply_octagon_rules(map, config, now);
}

}  // namespace vortex::mission

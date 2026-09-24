#include "landmark_server/map_rules.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vortex/utils/math.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::mission {

namespace {

using LT = vortex_msgs::msg::LandmarkType;
using LS = vortex_msgs::msg::LandmarkSubtype;
using vortex::filtering::LandmarkClassKey;
using vortex::utils::math::ssa;

constexpr double kDeg = M_PI / 180.0;

Eigen::Quaterniond yaw_quaternion(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

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

/**
 * Feed a yaw estimate into a landmark's yaw state: the first estimate sets the
 * yaw, later ones must agree with the running mean to count; N consistent
 * estimates lock it. Estimates that jump too far from the current yaw are
 * ignored.
 */
void feed_yaw_estimate(RetainedLandmark& lm,
                       double estimate,
                       const MapRulesConfig& cfg) {
    if (lm.yaw_locked) {
        return;
    }
    if (!lm.has_orientation) {
        lm.orientation = yaw_quaternion(estimate);
        lm.has_orientation = true;
        lm.yaw_sum_sin = std::sin(estimate);
        lm.yaw_sum_cos = std::cos(estimate);
        lm.yaw_count = 1;
        return;
    }
    const double current = lm.yaw();
    const double jump = std::abs(ssa(estimate - current));
    if (jump > cfg.yaw_max_jump_deg * kDeg) {
        return;
    }
    if (jump <= cfg.yaw_agree_deg * kDeg) {
        lm.yaw_sum_sin += std::sin(estimate);
        lm.yaw_sum_cos += std::cos(estimate);
        ++lm.yaw_count;
    } else {
        // Disagrees: start a new run from this estimate.
        lm.yaw_sum_sin = std::sin(estimate);
        lm.yaw_sum_cos = std::cos(estimate);
        lm.yaw_count = 1;
    }
    lm.orientation = yaw_quaternion(std::atan2(lm.yaw_sum_sin, lm.yaw_sum_cos));
    if (lm.yaw_count >= cfg.yaw_lock_consistent_estimates) {
        lm.yaw_locked = true;
    }
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

void apply_gate_rules(RetainedLandmarks& map,
                      const MapRulesConfig& cfg,
                      const Eigen::Vector3d& vehicle,
                      double now,
                      CourseFrameTracker* course) {
    RetainedLandmark* survey =
        find_measured(map, {LT::GATE, LS::GATE_SURVEY_REPAIR});
    RetainedLandmark* rescue =
        find_measured(map, {LT::GATE, LS::GATE_SEARCH_RESCUE});
    if (survey == nullptr || rescue == nullptr) {
        return;
    }

    const Eigen::Vector2d a = survey->position.head<2>();
    const Eigen::Vector2d b = rescue->position.head<2>();
    if ((b - a).norm() < cfg.min_panel_separation_m) {
        return;
    }
    const Eigen::Vector3d midpoint =
        0.5 * (survey->position + rescue->position);
    const double last_seen =
        std::max(survey->last_measurement, rescue->last_measurement);

    // The gate itself: measured if perception gave it, else synthetic.
    RetainedLandmark* gate = find_measured(map, {LT::GATE, LS::GATE_WHOLE});
    absorb_derived_into_measured(map, "gate_whole", gate);
    if (gate == nullptr) {
        gate =
            &map.upsert_derived("gate_whole", {LT::GATE, LS::GATE_WHOLE}, now);
        gate->last_measurement = last_seen;
        gate->derived_live = is_fresh(*survey) || is_fresh(*rescue);
    }
    gate->position = midpoint;

    // Yaw from the panels. Estimates are only taken while both panels are
    // being seen; the front is the side the vehicle first saw it from.
    if (cfg.gate_yaw_from_panels && is_fresh(*survey) && is_fresh(*rescue)) {
        const Eigen::Vector2d reference =
            gate->has_orientation
                ? Eigen::Vector2d(std::cos(gate->yaw()), std::sin(gate->yaw()))
                : Eigen::Vector2d((vehicle - midpoint).head<2>());
        const double estimate =
            yaw_of_direction(normal_towards(a, b, reference));
        feed_yaw_estimate(*gate, estimate, cfg);

        if (course != nullptr && gate->has_orientation) {
            course->add_gate_estimate(midpoint.head<2>(), gate->yaw());
        }
    }

    // The panels inherit the yaw of the gate.
    if (gate->has_orientation) {
        for (RetainedLandmark* panel : {survey, rescue}) {
            panel->orientation = gate->orientation;
            panel->has_orientation = true;
            panel->yaw_locked = gate->yaw_locked;
        }
    }
}

// --- Torpedo board ----------------------------------------------------------

struct Icon {
    RetainedLandmark* lm{nullptr};
    explicit operator bool() const { return lm != nullptr; }
    Eigen::Vector3d p() const { return lm->position; }
};

void apply_board_rules(RetainedLandmarks& map,
                       const MapRulesConfig& cfg,
                       const Eigen::Vector3d& vehicle,
                       double now) {
    Icon fire{find_measured(map, {LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRE})};
    Icon blood{find_measured(map, {LT::TORPEDO_BOARD, LS::TORPEDO_ICON_BLOOD})};
    Icon truck{
        find_measured(map, {LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRETRUCK})};
    Icon ambulance{
        find_measured(map, {LT::TORPEDO_BOARD, LS::TORPEDO_ICON_AMBULANCE})};

    std::vector<Icon> icons;
    for (const Icon& i : {fire, blood, truck, ambulance}) {
        if (i) {
            icons.push_back(i);
        }
    }
    if (icons.size() < 2) {
        return;
    }

    Eigen::Vector3d centre = Eigen::Vector3d::Zero();
    double last_seen = 0.0;
    bool all_fresh = true;
    for (const Icon& i : icons) {
        centre += i.p();
        last_seen = std::max(last_seen, i.lm->last_measurement);
        all_fresh = all_fresh && is_fresh(*i.lm);
    }
    centre /= static_cast<double>(icons.size());

    RetainedLandmark* board =
        find_measured(map, {LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE});
    absorb_derived_into_measured(map, "board_whole", board);
    if (board == nullptr) {
        board = &map.upsert_derived(
            "board_whole", {LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE}, now);
        board->last_measurement = last_seen;
        board->derived_live =
            std::any_of(icons.begin(), icons.end(),
                        [](const Icon& i) { return is_fresh(*i.lm); });
    }
    // The board is pulled to the centre of its icons.
    board->position = centre;

    // Yaw from the icon pairs. Each pair with enough horizontal spread gives
    // a normal; the normals of both pairs are added, so noise in one pair is
    // averaged with the other.
    if (cfg.board_yaw_from_icons && all_fresh) {
        struct Pair {
            Icon a;
            Icon b;
        };
        const Eigen::Vector2d reference =
            board->has_orientation
                ? Eigen::Vector2d(std::cos(board->yaw()),
                                  std::sin(board->yaw()))
                : Eigen::Vector2d((vehicle - centre).head<2>());
        Eigen::Vector2d fused = Eigen::Vector2d::Zero();
        for (const Pair& pair : {Pair{ambulance, truck}, Pair{fire, blood}}) {
            if (!pair.a || !pair.b) {
                continue;
            }
            const Eigen::Vector2d pa = pair.a.p().head<2>();
            const Eigen::Vector2d pb = pair.b.p().head<2>();
            if ((pb - pa).norm() < cfg.min_icon_separation_m) {
                continue;
            }
            fused += normal_towards(pa, pb, reference);
        }
        if (fused.norm() > 1e-6) {
            feed_yaw_estimate(*board, yaw_of_direction(fused), cfg);
        }
    }

    if (!board->has_orientation) {
        return;
    }

    // Board version from the icon heights: fire above blood = version 1
    // (z is down, so "above" is a smaller z).
    if (!cfg.torpedo_targets_from_icons || !fire || !blood) {
        return;
    }
    const bool version_1 = fire.p().z() < blood.p().z();
    const TorpedoIconOffsets& offsets =
        version_1 ? cfg.torpedo_version_1 : cfg.torpedo_version_2;

    struct Target {
        Icon icon;
        uint16_t subtype;
        const char* slot;
        Eigen::Vector3d offset;
    };
    const Target targets[] = {
        {fire, LS::TORPEDO_TARGET_LARGE_SURVEY_REPAIR,
         "torpedo_target_large_survey_repair", offsets.fire},
        {blood, LS::TORPEDO_TARGET_LARGE_SEARCH_RESCUE,
         "torpedo_target_large_search_rescue", offsets.blood},
        {truck, LS::TORPEDO_TARGET_SMALL_SURVEY_REPAIR,
         "torpedo_target_small_survey_repair", offsets.firetruck},
        {ambulance, LS::TORPEDO_TARGET_SMALL_SEARCH_RESCUE,
         "torpedo_target_small_search_rescue", offsets.ambulance}};
    const Eigen::Quaterniond q = board->orientation;
    for (const Target& t : targets) {
        if (!t.icon) {
            continue;
        }
        RetainedLandmark& target =
            map.upsert_derived(t.slot, {LT::TORPEDO_BOARD, t.subtype}, now);
        target.position = t.icon.p() + q * t.offset;
        target.orientation = q;
        target.has_orientation = true;
        target.yaw_locked = board->yaw_locked;
        target.last_measurement = t.icon.lm->last_measurement;
        target.derived_live = is_fresh(*t.icon.lm);
    }
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
    RetainedLandmark* table = find_measured(map, {LT::TABLE, LS::TABLE_WHOLE});
    if (table == nullptr) {
        return;
    }
    // The octagon is over the table: same xy. It floats at the water surface,
    // the table stands on the floor.
    RetainedLandmark* octagon =
        find_measured(map, {LT::OCTAGON, LS::OCTAGON_WHOLE});
    absorb_derived_into_measured(map, "octagon_whole", octagon);
    if (octagon == nullptr) {
        octagon = &map.upsert_derived("octagon_whole",
                                      {LT::OCTAGON, LS::OCTAGON_WHOLE}, now);
        octagon->last_measurement = table->last_measurement;
        octagon->derived_live = is_fresh(*table);
    }
    octagon->position = table->position;
    if (cfg.z_lock.enable) {
        octagon->position.z() = cfg.z_lock.surface_z;
    }
}

}  // namespace

void apply_map_rules(RetainedLandmarks& map,
                     const MapRulesConfig& config,
                     const Eigen::Vector3d& vehicle_position,
                     double now,
                     CourseFrameTracker* course) {
    apply_gate_rules(map, config, vehicle_position, now, course);
    apply_board_rules(map, config, vehicle_position, now);
    apply_bin_rules(map, config);
    apply_octagon_rules(map, config, now);
}

}  // namespace vortex::mission

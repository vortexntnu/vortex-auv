#include "landmark_server/class_config.hpp"
#include "landmark_server/landmark_classes.hpp"
#include <algorithm>
#include <stdexcept>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::mission {

namespace {


// The names of the classes, generated from vortex_msgs at build time
// (scripts/generate_class_names.py).
using generated::kSubtypes;
using generated::kTypes;

template <typename T>
T get_or(const YAML::Node& node, const char* key, T fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}

/// Apply the fields of a `classes.<NAME>` entry to a rule. `subtype` selects
/// the entry of a per-subtype `max_instances` map.
void apply_rule_fields(const YAML::Node& node,
                       uint16_t type,
                       uint16_t subtype,
                       ClassRule& rule) {
    static const std::vector<std::string> kKeys = {
        "max_instances", "instance_gate_m", "retain", "retain_sec",
        "keep_after_observations", "live_only"};
    for (const auto& kv : node) {
        const auto key = kv.first.as<std::string>();
        if (std::find(kKeys.begin(), kKeys.end(), key) == kKeys.end()) {
            throw std::runtime_error("classes: unknown key '" + key + "'");
        }
    }
    if (node["max_instances"]) {
        const auto& mi = node["max_instances"];
        if (mi.IsMap()) {
            for (const auto& kv : mi) {
                const auto sub =
                    parse_landmark_subtype(type, kv.first.as<std::string>());
                if (!sub) {
                    throw std::runtime_error(
                        "Unknown subtype in max_instances: " +
                        kv.first.as<std::string>());
                }
                if (subtype == *sub) {
                    rule.max_instances = kv.second.as<int>();
                }
            }
        } else {
            rule.max_instances = mi.as<int>();
        }
    }
    rule.instance_gate_m =
        get_or<double>(node, "instance_gate_m", rule.instance_gate_m);
    if (node["retain"]) {
        rule.retain_forever = node["retain"].as<std::string>() == "forever";
    }
    if (node["retain_sec"]) {
        rule.retain_sec = node["retain_sec"].as<double>();
        rule.retain_forever = false;
    }
    rule.keep_after_observations = get_or<int>(node, "keep_after_observations",
                                               rule.keep_after_observations);
    rule.live_only = get_or<bool>(node, "live_only", rule.live_only);
}

}  // namespace

std::optional<uint16_t> parse_landmark_type(const std::string& name) {
    for (const auto& t : kTypes) {
        if (name == t.name) {
            return t.value;
        }
    }
    return std::nullopt;
}

std::optional<uint16_t> parse_landmark_subtype(uint16_t type,
                                               const std::string& name) {
    for (const auto& s : kSubtypes) {
        if (s.type == type && (name == s.short_name || name == s.full)) {
            return s.value;
        }
    }
    return std::nullopt;
}

std::string class_name(const LandmarkClassKey& key) {
    for (const auto& s : kSubtypes) {
        if (s.type == key.type && s.value == key.subtype) {
            return s.full;
        }
    }
    for (const auto& t : kTypes) {
        if (t.value == key.type) {
            return key.subtype == 0 ? std::string(t.name)
                                    : std::string(t.name) + "/" +
                                          std::to_string(key.subtype);
        }
    }
    return std::to_string(key.type) + "/" + std::to_string(key.subtype);
}

std::vector<uint16_t> known_subtypes(uint16_t type) {
    std::vector<uint16_t> out;
    for (const auto& s : kSubtypes) {
        if (s.type == type) {
            out.push_back(s.value);
        }
    }
    return out;
}

std::optional<std::pair<uint16_t, uint16_t>> parse_class_name(
    const std::string& name) {
    if (const auto type = parse_landmark_type(name)) {
        return std::make_pair(*type, uint16_t{0});
    }
    for (const auto& s : kSubtypes) {
        if (name == s.full) {
            return std::make_pair(s.type, s.value);
        }
    }
    return std::nullopt;
}

namespace {

bool in_class_list(const std::vector<std::pair<uint16_t, uint16_t>>& list,
                   const LandmarkClassKey& key) {
    return std::any_of(list.begin(), list.end(), [&](const auto& c) {
        return c.first == key.type &&
               (c.second == 0 || c.second == key.subtype);
    });
}

}  // namespace

Eigen::Matrix3d DetectorNoise::covariance(const Eigen::Vector3d& ray) const {
    const double d = ray.norm();
    const double along = base_std_m + along_std_per_m * d;
    const double across = base_std_m + across_std_per_m * d;
    const Eigen::Vector3d u =
        d > 1e-6 ? Eigen::Vector3d(ray / d) : Eigen::Vector3d::UnitX();
    return Eigen::Matrix3d::Identity() * across * across +
           (along * along - across * across) * u * u.transpose();
}

bool ZLockConfig::is_floor(const LandmarkClassKey& key) const {
    return enable && in_class_list(floor_classes, key);
}

bool ZLockConfig::is_surface(const LandmarkClassKey& key) const {
    return enable && in_class_list(surface_classes, key);
}

const ClassRule& LandmarkMapConfig::rule_for(
    const LandmarkClassKey& key) const {
    auto it = class_rules.find({key.type, key.subtype});
    if (it == class_rules.end()) {
        it = class_rules.find({key.type, uint16_t{0}});
    }
    return it == class_rules.end() ? default_rule : it->second;
}

const MarkerBox* LandmarkMapConfig::marker_box_for(
    const LandmarkClassKey& key) const {
    for (const auto& k : {std::make_pair(key.type, key.subtype),
                          std::make_pair(key.type, uint16_t{0})}) {
        const auto it = marker_boxes.find(k);
        if (it != marker_boxes.end()) {
            return &it->second;
        }
    }
    return nullptr;
}

LandmarkMapConfig parse_map_config(const YAML::Node& root) {
    LandmarkMapConfig cfg;
    if (!root) {
        return cfg;
    }

    if (const auto noise = root["detector_noise"]) {
        DetectorNoise& n = cfg.intake.noise;
        n.base_std_m = get_or<double>(noise, "base_std_m", n.base_std_m);
        n.along_std_per_m =
            get_or<double>(noise, "along_std_per_m", n.along_std_per_m);
        n.across_std_per_m =
            get_or<double>(noise, "across_std_per_m", n.across_std_per_m);
        if (n.base_std_m < 0.0 || n.along_std_per_m < 0.0 ||
            n.across_std_per_m < 0.0) {
            throw std::runtime_error("detector_noise values must be >= 0");
        }
    }
    if (const auto intake = root["intake"]) {
        for (const char* gone : {"max_pipe_distance_m", "distance_noise"}) {
            if (intake[gone]) {
                throw std::runtime_error(
                    std::string("intake.") + gone +
                    " is gone: see detector_noise and the course tasks' "
                    "max_range_m");
            }
        }
        cfg.intake.no_orientation_rot_variance =
            get_or<double>(intake, "no_orientation_rot_variance",
                           cfg.intake.no_orientation_rot_variance);
        if (const auto mc = intake["measurement_covariance"]) {
            cfg.intake.use_measurement_covariance =
                get_or<bool>(mc, "use", cfg.intake.use_measurement_covariance);
            cfg.intake.covariance_scale =
                get_or<double>(mc, "scale", cfg.intake.covariance_scale);
            cfg.intake.covariance_min_std_m = get_or<double>(
                mc, "min_std_m", cfg.intake.covariance_min_std_m);
        }
    }

    if (const auto cf = root["course_frame"]) {
        cfg.course_frame.publish_tf =
            get_or<bool>(cf, "publish_tf", cfg.course_frame.publish_tf);
        cfg.course_frame.frame_id =
            get_or<std::string>(cf, "frame_id", cfg.course_frame.frame_id);
        cfg.course_frame.warn_start_vs_gate_deg =
            get_or<double>(cf, "warn_start_vs_gate_deg",
                           cfg.course_frame.warn_start_vs_gate_deg);
        if (const auto lock = cf["gate_lock"]) {
            cfg.course_frame.gate_lock_consistent_estimates =
                get_or<int>(lock, "consistent_estimates",
                            cfg.course_frame.gate_lock_consistent_estimates);
            cfg.course_frame.gate_lock_max_yaw_std_deg =
                get_or<double>(lock, "max_yaw_std_deg",
                               cfg.course_frame.gate_lock_max_yaw_std_deg);
        }
        if (cf["lane"]) {
            throw std::runtime_error(
                "course_frame.lane is gone: the lane is the area the course "
                "layout covers (course.lane_margin_m)");
        }
    }

    if (const auto classes = root["classes"]) {
        // Type-level entries first, then subtype-specific ones override.
        for (int pass = 0; pass < 2; ++pass) {
            for (const auto& kv : classes) {
                const std::string name = kv.first.as<std::string>();
                const auto parsed = parse_class_name(name);
                if (!parsed) {
                    throw std::runtime_error("Unknown landmark class: " + name);
                }
                const bool subtype_entry = parsed->second != 0;
                if (subtype_entry != (pass == 1)) {
                    continue;
                }
                std::vector<uint16_t> subtypes =
                    subtype_entry ? std::vector<uint16_t>{parsed->second}
                                  : known_subtypes(parsed->first);
                if (!subtype_entry) {
                    subtypes.push_back(0);
                }
                for (const uint16_t sub : subtypes) {
                    auto key = std::make_pair(parsed->first, sub);
                    auto it = cfg.class_rules.find(key);
                    ClassRule rule = it == cfg.class_rules.end()
                                         ? cfg.default_rule
                                         : it->second;
                    apply_rule_fields(kv.second, parsed->first, sub, rule);
                    cfg.class_rules[key] = rule;
                }
            }
        }
    }

    cfg.course = parse_course_config(root["course"]);

    if (const auto rules = root["rules"]) {
        cfg.plausibility_radius_m = get_or<double>(
            rules, "plausibility_radius_m", cfg.plausibility_radius_m);
        if (const auto adoption = rules["adoption"]) {
            cfg.adoption_ambiguity_ratio = get_or<double>(
                adoption, "ambiguity_ratio", cfg.adoption_ambiguity_ratio);
            cfg.adoption_wait_sec =
                get_or<double>(adoption, "wait_sec", cfg.adoption_wait_sec);
        }
        MapRulesConfig& mr = cfg.map_rules;
        for (const char* gone :
             {"gate_yaw_from_panels", "board_yaw_from_icons", "yaw_lock",
              "board", "torpedo_targets_from_icons", "large_structures",
              "large_structure_separation_m"}) {
            if (rules[gone]) {
                throw std::runtime_error(
                    std::string("rules.") + gone +
                    " is gone: the gate and the torpedo board come from their "
                    "course templates");
            }
        }
        mr.bin_role_from_down_icons = get_or<bool>(
            rules, "bin_role_from_down_icons", mr.bin_role_from_down_icons);
        mr.octagon_from_table =
            get_or<bool>(rules, "octagon_from_table", mr.octagon_from_table);
        mr.bin_role_radius_m =
            get_or<double>(rules, "bin_role_radius_m", mr.bin_role_radius_m);
        mr.min_panel_separation_m = get_or<double>(
            rules, "min_panel_separation_m", mr.min_panel_separation_m);
        mr.max_panel_separation_m = get_or<double>(
            rules, "max_panel_separation_m", mr.max_panel_separation_m);
        if (const auto to = rules["table_octagon"]) {
            mr.table_octagon_primary =
                get_or<std::string>(to, "primary", mr.table_octagon_primary);
            if (mr.table_octagon_primary != "table" &&
                mr.table_octagon_primary != "octagon" &&
                mr.table_octagon_primary != "midpoint") {
                throw std::runtime_error(
                    "rules.table_octagon.primary must be table, octagon or "
                    "midpoint, not '" +
                    mr.table_octagon_primary + "'");
            }
            mr.table_height_m =
                get_or<double>(to, "table_height_m", mr.table_height_m);
        }
        if (const auto z = rules["z_lock"]) {
            const auto classes =
                [&](const char* key,
                    std::vector<std::pair<uint16_t, uint16_t>>& out) {
                    if (!z[key]) {
                        return;
                    }
                    for (const auto& item : z[key]) {
                        const auto parsed =
                            parse_class_name(item.as<std::string>());
                        if (!parsed) {
                            throw std::runtime_error(
                                "Unknown class in z_lock: " +
                                item.as<std::string>());
                        }
                        out.push_back(*parsed);
                    }
                };
            mr.z_lock.enable = get_or<bool>(z, "enable", mr.z_lock.enable);
            mr.z_lock.floor_z = get_or<double>(z, "floor_z", mr.z_lock.floor_z);
            mr.z_lock.surface_z =
                get_or<double>(z, "surface_z", mr.z_lock.surface_z);
            classes("floor_classes", mr.z_lock.floor_classes);
            classes("surface_classes", mr.z_lock.surface_classes);
        }
    }

    if (const auto markers = root["markers"]) {
        for (const auto& kv : markers["boxes"]) {
            const auto name = kv.first.as<std::string>();
            const auto parsed = parse_class_name(name);
            if (!parsed) {
                throw std::runtime_error("Unknown class in markers.boxes: " +
                                         name);
            }
            const auto vec3 = [&](const char* field) {
                const auto n = kv.second[field];
                if (!n) {
                    return Eigen::Vector3d::Zero().eval();
                }
                if (!n.IsSequence() || n.size() != 3) {
                    throw std::runtime_error("markers.boxes." + name + "." +
                                             field + " must be [x, y, z]");
                }
                return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(),
                                       n[2].as<double>());
            };
            MarkerBox box;
            box.size = vec3("size");
            box.offset = vec3("offset");
            box.solid = get_or<bool>(kv.second, "solid", false);
            if (kv.second["color"]) {
                box.color = vec3("color");
            }
            cfg.marker_boxes[*parsed] = box;
        }
    }
    return cfg;
}

std::vector<std::pair<LandmarkClassKey, vortex::filtering::LandmarkClassConfig>>
parse_per_class_track_config(
    const YAML::Node& track_config,
    const vortex::filtering::LandmarkClassConfig& default_config) {
    using vortex::filtering::LandmarkClassConfig;
    std::vector<std::pair<LandmarkClassKey, LandmarkClassConfig>> out;
    if (!track_config) {
        return out;
    }

    const auto apply = [](const YAML::Node& n, LandmarkClassConfig& c) {
        if (const auto nm = n["nm"]) {
            c.nm.confirm_n = get_or<int>(nm, "confirm_n", c.nm.confirm_n);
            c.nm.confirm_m = get_or<int>(nm, "confirm_m", c.nm.confirm_m);
            c.nm.delete_n = get_or<int>(nm, "delete_n", c.nm.delete_n);
            c.nm.delete_m = get_or<int>(nm, "delete_m", c.nm.delete_m);
        }
        if (const auto gate = n["gate"]) {
            c.min_pos_error =
                get_or<double>(gate, "min_pos_error", c.min_pos_error);
            c.max_pos_error =
                get_or<double>(gate, "max_pos_error", c.max_pos_error);
            c.min_ori_error =
                get_or<double>(gate, "min_ori_error", c.min_ori_error);
            c.max_ori_error =
                get_or<double>(gate, "max_ori_error", c.max_ori_error);
        }
        c.dyn_std_dev = get_or<double>(n, "dyn_mod_std_dev", c.dyn_std_dev);
        c.sens_std_dev = get_or<double>(n, "sens_mod_std_dev", c.sens_std_dev);
        c.init_pos_std = get_or<double>(n, "init_pos_std_dev", c.init_pos_std);
        c.init_ori_std = get_or<double>(n, "init_ori_std_dev", c.init_ori_std);
        c.mahalanobis_threshold = get_or<double>(
            n, "mahalanobis_gate_threshold", c.mahalanobis_threshold);
        c.prob_of_detection =
            get_or<double>(n, "prob_of_detection", c.prob_of_detection);
        c.clutter_intensity =
            get_or<double>(n, "clutter_intensity", c.clutter_intensity);
        c.new_track_min_distance = get_or<double>(n, "new_track_min_distance_m",
                                                  c.new_track_min_distance);
        c.max_tracks = get_or<int>(n, "max_tracks", c.max_tracks);
    };

    for (int pass = 0; pass < 2; ++pass) {
        for (const auto& kv : track_config) {
            const std::string name = kv.first.as<std::string>();
            if (name == "default") {
                continue;
            }
            const auto parsed = parse_class_name(name);
            if (!parsed) {
                throw std::runtime_error(
                    "Unknown landmark class in track_config: " + name);
            }
            const bool subtype_entry = parsed->second != 0;
            if (subtype_entry != (pass == 1)) {
                continue;
            }
            std::vector<uint16_t> subtypes =
                subtype_entry ? std::vector<uint16_t>{parsed->second}
                              : known_subtypes(parsed->first);

            for (const uint16_t sub : subtypes) {
                const LandmarkClassKey key{parsed->first, sub};
                auto it =
                    std::find_if(out.begin(), out.end(),
                                 [&](const auto& e) { return e.first == key; });
                if (it == out.end()) {
                    out.emplace_back(key, default_config);
                    it = std::prev(out.end());
                }
                apply(kv.second, it->second);
            }
        }
    }
    return out;
}

}  // namespace vortex::mission

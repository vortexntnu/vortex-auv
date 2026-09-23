#include "landmark_server/class_config.hpp"
#include <algorithm>
#include <stdexcept>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::mission {

namespace {

using LT = vortex_msgs::msg::LandmarkType;
using LS = vortex_msgs::msg::LandmarkSubtype;

struct TypeName {
    const char* name;
    uint16_t value;
};

struct SubtypeName {
    uint16_t type;
    const char* full;
    const char* short_name;
    uint16_t value;
};

const TypeName kTypes[] = {{"ARUCO_MARKER", LT::ARUCO_MARKER},
                           {"ARUCO_BOARD", LT::ARUCO_BOARD},
                           {"PIPELINE_START", LT::PIPELINE_START},
                           {"PIPELINE_END", LT::PIPELINE_END},
                           {"VALVE", LT::VALVE},
                           {"GATE", LT::GATE},
                           {"SLALOM_PIPE", LT::SLALOM_PIPE},
                           {"TORPEDO_BOARD", LT::TORPEDO_BOARD},
                           {"BIN", LT::BIN},
                           {"PATH_MARKER", LT::PATH_MARKER},
                           {"TABLE", LT::TABLE},
                           {"OCTAGON", LT::OCTAGON},
                           {"PINGER", LT::PINGER}};

const SubtypeName kSubtypes[] = {
    {LT::VALVE, "VALVE_VERTICAL", "VERTICAL", LS::VALVE_VERTICAL},
    {LT::VALVE, "VALVE_HORIZONTAL", "HORIZONTAL", LS::VALVE_HORIZONTAL},
    {LT::PIPELINE_START, "PIPELINE_START_CAMERA", "CAMERA",
     LS::PIPELINE_START_CAMERA},
    {LT::PIPELINE_START, "PIPELINE_START_SONAR", "SONAR",
     LS::PIPELINE_START_SONAR},
    {LT::GATE, "GATE_SEARCH_RESCUE", "SEARCH_RESCUE", LS::GATE_SEARCH_RESCUE},
    {LT::GATE, "GATE_SURVEY_REPAIR", "SURVEY_REPAIR", LS::GATE_SURVEY_REPAIR},
    {LT::GATE, "GATE_WHOLE", "WHOLE", LS::GATE_WHOLE},
    {LT::SLALOM_PIPE, "SLALOM_PIPE_WHITE", "WHITE", LS::SLALOM_PIPE_WHITE},
    {LT::SLALOM_PIPE, "SLALOM_PIPE_RED", "RED", LS::SLALOM_PIPE_RED},
    {LT::TORPEDO_BOARD, "TORPEDO_BOARD_WHOLE", "WHOLE",
     LS::TORPEDO_BOARD_WHOLE},
    {LT::TORPEDO_BOARD, "TORPEDO_TARGET_LARGE_SEARCH_RESCUE",
     "TARGET_LARGE_SEARCH_RESCUE", LS::TORPEDO_TARGET_LARGE_SEARCH_RESCUE},
    {LT::TORPEDO_BOARD, "TORPEDO_TARGET_LARGE_SURVEY_REPAIR",
     "TARGET_LARGE_SURVEY_REPAIR", LS::TORPEDO_TARGET_LARGE_SURVEY_REPAIR},
    {LT::TORPEDO_BOARD, "TORPEDO_TARGET_SMALL_SEARCH_RESCUE",
     "TARGET_SMALL_SEARCH_RESCUE", LS::TORPEDO_TARGET_SMALL_SEARCH_RESCUE},
    {LT::TORPEDO_BOARD, "TORPEDO_TARGET_SMALL_SURVEY_REPAIR",
     "TARGET_SMALL_SURVEY_REPAIR", LS::TORPEDO_TARGET_SMALL_SURVEY_REPAIR},
    {LT::TORPEDO_BOARD, "TORPEDO_ICON_FIRE", "ICON_FIRE",
     LS::TORPEDO_ICON_FIRE},
    {LT::TORPEDO_BOARD, "TORPEDO_ICON_BLOOD", "ICON_BLOOD",
     LS::TORPEDO_ICON_BLOOD},
    {LT::TORPEDO_BOARD, "TORPEDO_ICON_FIRETRUCK", "ICON_FIRETRUCK",
     LS::TORPEDO_ICON_FIRETRUCK},
    {LT::TORPEDO_BOARD, "TORPEDO_ICON_AMBULANCE", "ICON_AMBULANCE",
     LS::TORPEDO_ICON_AMBULANCE},
    {LT::BIN, "BIN_SEARCH_RESCUE", "SEARCH_RESCUE", LS::BIN_SEARCH_RESCUE},
    {LT::BIN, "BIN_SURVEY_REPAIR", "SURVEY_REPAIR", LS::BIN_SURVEY_REPAIR},
    {LT::BIN, "BIN_UNCLASSIFIED", "UNCLASSIFIED", LS::BIN_UNCLASSIFIED},
    {LT::BIN, "BIN_STRUCTURE", "STRUCTURE", LS::BIN_STRUCTURE},
    {LT::PATH_MARKER, "PATH_MARKER_WHOLE", "WHOLE", LS::PATH_MARKER_WHOLE},
    {LT::TABLE, "TABLE_WHOLE", "WHOLE", LS::TABLE_WHOLE},
    {LT::TABLE, "TABLE_ITEM_NUTBOLT", "ITEM_NUTBOLT", LS::TABLE_ITEM_NUTBOLT},
    {LT::TABLE, "TABLE_ITEM_ELECTRIC", "ITEM_ELECTRIC",
     LS::TABLE_ITEM_ELECTRIC},
    {LT::TABLE, "TABLE_ITEM_PILL", "ITEM_PILL", LS::TABLE_ITEM_PILL},
    {LT::TABLE, "TABLE_ITEM_BANDAID", "ITEM_BANDAID", LS::TABLE_ITEM_BANDAID},
    {LT::TABLE, "TABLE_BASKET_SURVEY_REPAIR", "BASKET_SURVEY_REPAIR",
     LS::TABLE_BASKET_SURVEY_REPAIR},
    {LT::TABLE, "TABLE_BASKET_SEARCH_RESCUE", "BASKET_SEARCH_RESCUE",
     LS::TABLE_BASKET_SEARCH_RESCUE},
    {LT::OCTAGON, "OCTAGON_WHOLE", "WHOLE", LS::OCTAGON_WHOLE},
    {LT::OCTAGON, "OCTAGON_IMAGE_COMPASS", "IMAGE_COMPASS",
     LS::OCTAGON_IMAGE_COMPASS},
    {LT::OCTAGON, "OCTAGON_IMAGE_TOOLS", "IMAGE_TOOLS",
     LS::OCTAGON_IMAGE_TOOLS},
    {LT::OCTAGON, "OCTAGON_IMAGE_LIFEBUOY", "IMAGE_LIFEBUOY",
     LS::OCTAGON_IMAGE_LIFEBUOY},
    {LT::OCTAGON, "OCTAGON_IMAGE_SOS", "IMAGE_SOS", LS::OCTAGON_IMAGE_SOS},
    {LT::PINGER, "PINGER_DEPLOY", "DEPLOY", LS::PINGER_DEPLOY},
    {LT::PINGER, "PINGER_RESTORE", "RESTORE", LS::PINGER_RESTORE}};

template <typename T>
T get_or(const YAML::Node& node, const char* key, T fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}

LaneBox parse_lane_box(const YAML::Node& node, const LaneBox& fallback) {
    LaneBox box = fallback;
    if (!node) {
        return box;
    }
    if (node["x"] && node["x"].size() == 2) {
        box.x_min = node["x"][0].as<double>();
        box.x_max = node["x"][1].as<double>();
    }
    if (node["y"] && node["y"].size() == 2) {
        box.y_min = node["y"][0].as<double>();
        box.y_max = node["y"][1].as<double>();
    }
    return box;
}

/// Apply the fields of a `classes.<NAME>` entry to a rule. `subtype` selects
/// the entry of a per-subtype `max_instances` map.
void apply_rule_fields(const YAML::Node& node,
                       uint16_t type,
                       uint16_t subtype,
                       ClassRule& rule) {
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
    rule.min_distance_to_large_structures_m =
        get_or<double>(node, "min_distance_to_large_structures_m",
                       rule.min_distance_to_large_structures_m);
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

bool LandmarkMapConfig::is_large_structure(const LandmarkClassKey& key) const {
    return std::any_of(large_structures.begin(), large_structures.end(),
                       [&](const auto& s) {
                           return s.first == key.type &&
                                  (s.second == 0 || s.second == key.subtype);
                       });
}

LandmarkMapConfig parse_map_config(const YAML::Node& root) {
    LandmarkMapConfig cfg;
    if (!root) {
        return cfg;
    }

    if (const auto intake = root["intake"]) {
        cfg.intake.max_pipe_distance_m = get_or<double>(
            intake, "max_pipe_distance_m", cfg.intake.max_pipe_distance_m);
        cfg.intake.no_orientation_rot_variance =
            get_or<double>(intake, "no_orientation_rot_variance",
                           cfg.intake.no_orientation_rot_variance);
        if (const auto noise = intake["distance_noise"]) {
            cfg.intake.noise_base_variance = get_or<double>(
                noise, "base_variance", cfg.intake.noise_base_variance);
            cfg.intake.noise_variance_per_meter =
                get_or<double>(noise, "variance_per_meter",
                               cfg.intake.noise_variance_per_meter);
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
        if (const auto lane = cf["lane"]) {
            cfg.course_frame.before_gate = parse_lane_box(
                lane["before_gate"], cfg.course_frame.before_gate);
            cfg.course_frame.after_gate =
                parse_lane_box(lane["after_gate"], cfg.course_frame.after_gate);
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

    if (const auto rules = root["rules"]) {
        cfg.plausibility_radius_m = get_or<double>(
            rules, "plausibility_radius_m", cfg.plausibility_radius_m);
        MapRulesConfig& mr = cfg.map_rules;
        mr.gate_yaw_from_panels = get_or<bool>(rules, "gate_yaw_from_panels",
                                               mr.gate_yaw_from_panels);
        mr.board_yaw_from_icons = get_or<bool>(rules, "board_yaw_from_icons",
                                               mr.board_yaw_from_icons);
        mr.bin_role_from_down_icons = get_or<bool>(
            rules, "bin_role_from_down_icons", mr.bin_role_from_down_icons);
        mr.octagon_from_table =
            get_or<bool>(rules, "octagon_from_table", mr.octagon_from_table);
        if (const auto lock = rules["yaw_lock"]) {
            mr.yaw_lock_consistent_estimates = get_or<int>(
                lock, "consistent_estimates", mr.yaw_lock_consistent_estimates);
            mr.yaw_max_jump_deg =
                get_or<double>(lock, "max_jump_deg", mr.yaw_max_jump_deg);
            mr.yaw_agree_deg =
                get_or<double>(lock, "agree_deg", mr.yaw_agree_deg);
        }
        mr.bin_role_radius_m =
            get_or<double>(rules, "bin_role_radius_m", mr.bin_role_radius_m);
        mr.min_icon_separation_m = get_or<double>(
            rules, "min_icon_separation_m", mr.min_icon_separation_m);
        mr.min_panel_separation_m = get_or<double>(
            rules, "min_panel_separation_m", mr.min_panel_separation_m);
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
        if (const auto targets = rules["torpedo_targets_from_icons"]) {
            const auto vec = [](const YAML::Node& n, Eigen::Vector3d& out) {
                if (n && n.size() == 3) {
                    out = Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(),
                                          n[2].as<double>());
                }
            };
            const auto load = [&](const YAML::Node& v, TorpedoIconOffsets& o) {
                if (!v) {
                    return;
                }
                vec(v["fire"], o.fire);
                vec(v["blood"], o.blood);
                vec(v["firetruck"], o.firetruck);
                vec(v["ambulance"], o.ambulance);
            };
            load(targets["version_1"], mr.torpedo_version_1);
            load(targets["version_2"], mr.torpedo_version_2);
        }
        if (const auto ls = rules["large_structures"]) {
            for (const auto& item : ls) {
                const auto parsed = parse_class_name(item.as<std::string>());
                if (!parsed) {
                    throw std::runtime_error("Unknown large structure: " +
                                             item.as<std::string>());
                }
                cfg.large_structures.push_back(*parsed);
            }
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

#include "landmark_server/config.hpp"

#include <fmt/format.h>

#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::landmark_server {

namespace {

struct NamedValue {
    std::string_view name;
    std::uint16_t value;
};

// The constants a config can name. The values are the messages' own, so a
// renumbered constant only needs a rebuild; a new one is a line here.
#define LANDMARK_TYPE(name)                         \
    NamedValue {                                    \
        #name, vortex_msgs::msg::LandmarkType::name \
    }
#define LANDMARK_SUBTYPE(name)                         \
    NamedValue {                                       \
        #name, vortex_msgs::msg::LandmarkSubtype::name \
    }

constexpr NamedValue kTypes[] = {
    LANDMARK_TYPE(ARUCO_MARKER),
    LANDMARK_TYPE(ARUCO_BOARD),
    LANDMARK_TYPE(PIPELINE_START),
    LANDMARK_TYPE(PIPELINE_END),
    LANDMARK_TYPE(VALVE),
    LANDMARK_TYPE(GATE),
    LANDMARK_TYPE(SLALOM_PIPE),
    LANDMARK_TYPE(TORPEDO_BOARD),
    LANDMARK_TYPE(BIN),
    LANDMARK_TYPE(PATH_MARKER),
    LANDMARK_TYPE(TABLE),
    LANDMARK_TYPE(OCTAGON),
    LANDMARK_TYPE(PINGER),
};

constexpr NamedValue kSubtypes[] = {
    LANDMARK_SUBTYPE(ARUCO_BOARD_CAMERA),
    LANDMARK_SUBTYPE(ARUCO_BOARD_SONAR),
    LANDMARK_SUBTYPE(ARUCO_BOARD_DETECTION),
    LANDMARK_SUBTYPE(VALVE_VERTICAL),
    LANDMARK_SUBTYPE(VALVE_HORIZONTAL),
    LANDMARK_SUBTYPE(PIPELINE_START_CAMERA),
    LANDMARK_SUBTYPE(PIPELINE_START_SONAR),
    LANDMARK_SUBTYPE(GATE_SEARCH_RESCUE),
    LANDMARK_SUBTYPE(GATE_SURVEY_REPAIR),
    LANDMARK_SUBTYPE(SLALOM_PIPE_WHITE),
    LANDMARK_SUBTYPE(SLALOM_PIPE_RED),
    LANDMARK_SUBTYPE(TORPEDO_BOARD_WHOLE),
    LANDMARK_SUBTYPE(TORPEDO_TARGET_LARGE_SEARCH_RESCUE),
    LANDMARK_SUBTYPE(TORPEDO_TARGET_LARGE_SURVEY_REPAIR),
    LANDMARK_SUBTYPE(TORPEDO_TARGET_SMALL_SEARCH_RESCUE),
    LANDMARK_SUBTYPE(TORPEDO_TARGET_SMALL_SURVEY_REPAIR),
    LANDMARK_SUBTYPE(BIN_SEARCH_RESCUE),
    LANDMARK_SUBTYPE(BIN_SURVEY_REPAIR),
    LANDMARK_SUBTYPE(GATE_WHOLE),
    LANDMARK_SUBTYPE(GATE_POLE_EDGE),
    LANDMARK_SUBTYPE(GATE_POLE_MIDDLE),
    LANDMARK_SUBTYPE(TORPEDO_ICON_FIRE),
    LANDMARK_SUBTYPE(TORPEDO_ICON_BLOOD),
    LANDMARK_SUBTYPE(TORPEDO_ICON_FIRETRUCK),
    LANDMARK_SUBTYPE(TORPEDO_ICON_AMBULANCE),
    LANDMARK_SUBTYPE(BIN_UNCLASSIFIED),
    LANDMARK_SUBTYPE(BIN_STRUCTURE),
    LANDMARK_SUBTYPE(PATH_MARKER_WHOLE),
    LANDMARK_SUBTYPE(TABLE_WHOLE),
    LANDMARK_SUBTYPE(TABLE_ITEM_NUTBOLT),
    LANDMARK_SUBTYPE(TABLE_ITEM_ELECTRIC),
    LANDMARK_SUBTYPE(TABLE_ITEM_PILL),
    LANDMARK_SUBTYPE(TABLE_ITEM_BANDAID),
    LANDMARK_SUBTYPE(TABLE_BASKET_SURVEY_REPAIR),
    LANDMARK_SUBTYPE(TABLE_BASKET_SEARCH_RESCUE),
    LANDMARK_SUBTYPE(OCTAGON_WHOLE),
    LANDMARK_SUBTYPE(OCTAGON_IMAGE_REPAIR),
    LANDMARK_SUBTYPE(OCTAGON_IMAGE_RESCUE),
    LANDMARK_SUBTYPE(OCTAGON_IMAGE_SEARCH),
    LANDMARK_SUBTYPE(OCTAGON_IMAGE_SURVEY),
    LANDMARK_SUBTYPE(PINGER_DEPLOY),
    LANDMARK_SUBTYPE(PINGER_RESTORE),
};

#undef LANDMARK_TYPE
#undef LANDMARK_SUBTYPE

template <std::size_t N>
std::optional<std::uint16_t> value_of(const NamedValue (&table)[N],
                                      const std::string& name) {
    for (const NamedValue& entry : table) {
        if (entry.name == name) {
            return entry.value;
        }
    }
    return std::nullopt;
}

}  // namespace

std::optional<std::uint16_t> landmark_type(const std::string& name) {
    return value_of(kTypes, name);
}

std::optional<std::uint16_t> landmark_subtype(const std::string& name) {
    return value_of(kSubtypes, name);
}

const ClassConfig* Config::find_class(std::uint16_t type,
                                      std::uint16_t subtype) const {
    for (const ClassConfig& c : classes) {
        if (c.type == type && c.subtype == subtype) {
            return &c;
        }
    }
    return nullptr;
}

const ClassConfig* Config::find_class(const std::string& name) const {
    for (const ClassConfig& c : classes) {
        if (c.name == name) {
            return &c;
        }
    }
    return nullptr;
}

void Config::validate() const {
    const auto fail = [](const std::string& reason) {
        throw std::runtime_error(reason);
    };
    if (classes.empty()) {
        fail("no classes configured");
    }
    std::set<std::pair<std::uint16_t, std::uint16_t>> seen;
    for (const ClassConfig& c : classes) {
        if (!seen.insert({c.type, c.subtype}).second) {
            fail(fmt::format(
                "class '{}': another class has the same type and subtype",
                c.name));
        }
        if (c.symmetry_deg < 0.0 || c.symmetry_deg > 360.0) {
            fail(fmt::format("class '{}': symmetry_deg must be in [0, 360]",
                             c.name));
        }
        if (c.prior_radius_m <= 0.0 || c.max_instances < 0) {
            fail(fmt::format(
                "class '{}': prior_radius_m > 0 and max_instances >= 0",
                c.name));
        }
    }
    const Params& p = params;
    if (p.gate_prob <= 0.0 || p.gate_prob >= 1.0) {
        fail("gate_prob must be in (0, 1)");
    }
    if (p.confirm_hits < 1 || p.confirm_window_s <= 0.0) {
        fail("confirm_hits >= 1 and confirm_window_s > 0");
    }
    if (p.keyframe_dist_m <= 0.0 || p.keyframe_time_s <= 0.0 ||
        p.max_messages_per_keyframe < 1 || p.max_merged_per_factor < 1) {
        fail("keyframe values and message limits must be positive");
    }
    if (!p.slalom.red_class.empty() && (!find_class(p.slalom.red_class) ||
                                        !find_class(p.slalom.white_class))) {
        fail("slalom.red_class / white_class: not a configured class");
    }
    if (!p.gate.panel_classes.empty()) {
        if (p.gate.panel_classes.size() != 2) {
            fail("gate.panel_classes: give two classes or none");
        }
        for (const auto& name : p.gate.panel_classes) {
            if (!find_class(name)) {
                fail(fmt::format("gate.panel_classes: no class '{}'", name));
            }
        }
    }
}

}  // namespace vortex::landmark_server

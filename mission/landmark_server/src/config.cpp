#include "landmark_server/config.hpp"

#include <fmt/format.h>

#include <set>
#include <stdexcept>
#include <utility>

namespace vortex::landmark_server {

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
            fail(fmt::format("class '{}': type {} subtype {} is another class",
                             c.name, c.type, c.subtype));
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

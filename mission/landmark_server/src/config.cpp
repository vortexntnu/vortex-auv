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
    if (p.max_range_m <= 0.0 || p.cluster_radius_m <= 0.0) {
        fail("max_range_m and cluster_radius_m must be positive");
    }
    if (p.confirm_hits < 1 || p.max_hits_per_class < p.confirm_hits) {
        fail("confirm_hits >= 1 and max_hits_per_class >= confirm_hits");
    }
    if (p.measure_duration_s <= 0.0 || p.measure_max_range_m <= 0.0 ||
        p.measure_min_hits < 1 || p.measure_max_spread_m <= 0.0 ||
        p.slalom_rows < 1 || p.slalom_spacing_m <= 0.0 ||
        p.slalom_max_row_error_m <= 0.0) {
        fail("measure values must be positive");
    }
    if (p.max_hit_age_s < 0.0) {
        fail("max_hit_age_s must be >= 0 (0 keeps every hit)");
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

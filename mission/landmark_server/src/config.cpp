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
        fail("max_range_m and measure.cluster_radius_m must be positive");
    }
    const AssociationParams& a = p.association;
    if (a.gate_prob <= 0.0 || a.gate_prob >= 1.0) {
        fail("association.gate_prob must be in (0, 1)");
    }
    if (a.candidate_radius_m <= 0.0 || a.confirm_hits < 1 ||
        a.confirm_window_s <= 0.0 || a.merge_radius_m <= 0.0) {
        fail(
            "association: radii, confirm_hits and the window must be positive");
    }
    const GraphParams& g = p.graph;
    if (g.keyframe_dist_m <= 0.0 || g.keyframe_time_s <= 0.0 ||
        g.min_sigma_trans <= 0.0 || g.min_sigma_yaw <= 0.0 ||
        g.sigma_roll_pitch <= 0.0 || g.sigma_z <= 0.0 ||
        g.attitude_sigma <= 0.0 || g.depth_sigma <= 0.0 ||
        g.bearing_sigma <= 0.0 || g.range_sigma_a <= 0.0 || g.dcs_phi <= 0.0) {
        fail("graph values must be positive");
    }
    if (p.measure_duration_s <= 0.0 || p.measure_max_range_m <= 0.0 ||
        p.measure_min_hits < 1 || p.measure_max_spread_m <= 0.0) {
        fail("measure values must be positive");
    }
}

}  // namespace vortex::landmark_server

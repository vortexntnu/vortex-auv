#include "landmark_slam/config.hpp"

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace vortex::landmark_slam {

namespace {

template <typename T>
T required(const YAML::Node& node, const char* key, const std::string& where) {
    if (!node[key]) {
        throw std::runtime_error(fmt::format("{}: missing '{}'", where, key));
    }
    return node[key].as<T>();
}

template <typename T>
T optional_or(const YAML::Node& node, const char* key, T fallback) {
    return node[key] ? node[key].as<T>() : fallback;
}

YAML::Node load_file(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error(fmt::format("no such file: '{}'", path));
    }
    return YAML::LoadFile(path);
}

std::vector<ClassConfig> load_classes(const std::string& path) {
    const YAML::Node root = load_file(path);
    std::vector<ClassConfig> classes;
    std::set<std::pair<std::uint16_t, std::uint16_t>> seen;
    for (const auto& entry : root) {
        ClassConfig c;
        c.name = entry.first.as<std::string>();
        const YAML::Node& n = entry.second;
        const std::string where = fmt::format("{}: class '{}'", path, c.name);
        c.type = required<std::uint16_t>(n, "type", where);
        c.subtype = required<std::uint16_t>(n, "subtype", where);
        c.symmetry_deg = optional_or<double>(n, "symmetry_deg", 0.0);
        c.has_orientation = optional_or<bool>(n, "has_orientation", false);
        if (c.symmetry_deg < 0.0 || c.symmetry_deg > 360.0) {
            throw std::runtime_error(
                fmt::format("{}: symmetry_deg must be in [0, 360]", where));
        }
        if (!seen.insert({c.type, c.subtype}).second) {
            throw std::runtime_error(
                fmt::format("{}: type {} subtype {} is already another class",
                            where, c.type, c.subtype));
        }
        classes.push_back(c);
    }
    if (classes.empty()) {
        throw std::runtime_error(fmt::format("{}: no classes", path));
    }
    return classes;
}

void load_prior_map(const std::string& path, Config& cfg) {
    const YAML::Node root = load_file(path);
    const Params& p = cfg.params;

    InitialPose init;
    if (const YAML::Node n = root["initial_pose"]) {
        init.x = optional_or<double>(n, "x", 0.0);
        init.y = optional_or<double>(n, "y", 0.0);
        if (n["z"]) {
            init.z = n["z"].as<double>();
        }
        init.yaw = optional_or<double>(n, "yaw", 0.0);
        init.sigma_xy = optional_or<double>(n, "sigma_xy", init.sigma_xy);
        init.sigma_yaw = optional_or<double>(n, "sigma_yaw", init.sigma_yaw);
    }
    cfg.initial_pose = init;

    std::set<int> ids;
    for (const auto& n : root["landmarks"]) {
        PriorLandmark l;
        const std::string where = fmt::format("{}: landmark", path);
        l.id = required<int>(n, "id", where);
        l.class_name = required<std::string>(n, "class", where);
        l.x = required<double>(n, "x", where);
        l.y = required<double>(n, "y", where);
        l.z = required<double>(n, "z", where);
        l.yaw = optional_or<double>(n, "yaw", 0.0);
        l.sigma_xy =
            optional_or<double>(n, "sigma_xy", p.default_prior_sigma_xy);
        l.sigma_z = optional_or<double>(n, "sigma_z", p.default_prior_sigma_z);
        l.sigma_yaw =
            optional_or<double>(n, "sigma_yaw", p.default_prior_sigma_yaw);
        bool known = false;
        for (const auto& c : cfg.classes) {
            known = known || c.name == l.class_name;
        }
        if (!known) {
            throw std::runtime_error(
                fmt::format("{} {}: class '{}' is not in the class file", where,
                            l.id, l.class_name));
        }
        if (!ids.insert(l.id).second) {
            throw std::runtime_error(
                fmt::format("{}: id {} used twice", where, l.id));
        }
        cfg.prior_landmarks.push_back(l);
    }

    // Two entries of one class closer than their uncertainty cannot be told
    // apart: a detection of one fits the other's prior as well.
    const auto& pl = cfg.prior_landmarks;
    for (std::size_t a = 0; a < pl.size(); ++a) {
        for (std::size_t b = a + 1; b < pl.size(); ++b) {
            const double d = std::hypot(pl[a].x - pl[b].x, pl[a].y - pl[b].y);
            if (pl[a].class_name == pl[b].class_name &&
                d < 2.0 * (pl[a].sigma_xy + pl[b].sigma_xy)) {
                spdlog::warn(
                    "landmark_slam: prior landmarks {} and {} ({}) are {:.2f} "
                    "m "
                    "apart with sigma_xy {:.2f} and {:.2f}: too loose to tell "
                    "them apart, use sigma_xy <= {:.2f}",
                    pl[a].id, pl[b].id, pl[a].class_name, d, pl[a].sigma_xy,
                    pl[b].sigma_xy, d / 4.0);
            }
        }
    }
}

}  // namespace

const ClassConfig* Config::find_class(std::uint16_t type,
                                      std::uint16_t subtype) const {
    for (const auto& c : classes) {
        if (c.type == type && c.subtype == subtype) {
            return &c;
        }
    }
    return nullptr;
}

Config load_config(const Params& params) {
    Config cfg;
    cfg.params = params;
    cfg.classes = load_classes(params.classes_file);
    if (params.use_prior_map && !params.prior_map_file.empty()) {
        load_prior_map(params.prior_map_file, cfg);
    }
    return cfg;
}

}  // namespace vortex::landmark_slam

#include "landmark_server/ros_config.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <vortex/utils/ros/landmark_names.hpp>

namespace vortex::landmark_server {

Config load_config(rclcpp::Node& node) {
    Config cfg;
    Params& p = cfg.params;
    const auto read = [&node](const std::string& name, auto& value) {
        using T = std::decay_t<decltype(value)>;
        value = node.get_parameter_or<T>(name, value);
    };

    read("max_range_m", p.max_range_m);

    AssociationParams& a = p.association;
    read("association.gate_prob", a.gate_prob);
    read("association.ambiguity_d2", a.ambiguity_d2);
    read("association.candidate_radius_m", a.candidate_radius_m);
    read("association.confirm_hits", a.confirm_hits);
    read("association.confirm_window_s", a.confirm_window_s);
    read("association.merge_radius_m", a.merge_radius_m);

    GraphParams& g = p.graph;
    read("graph.keyframe_dist_m", g.keyframe_dist_m);
    read("graph.keyframe_time_s", g.keyframe_time_s);
    read("graph.sigma_trans_per_m", g.sigma_trans_per_m);
    read("graph.sigma_yaw_per_m", g.sigma_yaw_per_m);
    read("graph.min_sigma_trans", g.min_sigma_trans);
    read("graph.min_sigma_yaw", g.min_sigma_yaw);
    read("graph.sigma_roll_pitch", g.sigma_roll_pitch);
    read("graph.sigma_z", g.sigma_z);
    read("graph.attitude_sigma", g.attitude_sigma);
    read("graph.depth_sigma", g.depth_sigma);
    read("graph.bearing_sigma", g.bearing_sigma);
    read("graph.range_sigma_a", g.range_sigma_a);
    read("graph.range_sigma_b", g.range_sigma_b);
    read("graph.dcs_phi", g.dcs_phi);

    read("measure.cluster_radius_m", p.cluster_radius_m);
    read("measure.duration_s", p.measure_duration_s);
    read("measure.max_range_m", p.measure_max_range_m);
    read("measure.min_hits", p.measure_min_hits);
    read("measure.max_spread_m", p.measure_max_spread_m);

    std::set<std::string> names;
    for (const auto& full : node.list_parameters({"classes"}, 3).names) {
        const auto first = full.find('.');
        const auto second = full.find('.', first + 1);
        if (first != std::string::npos && second != std::string::npos) {
            names.insert(full.substr(first + 1, second - first - 1));
        }
    }
    for (const auto& name : names) {
        const std::string k = "classes." + name + ".";
        ClassConfig c;
        c.name = name;
        std::string type;
        std::string subtype;
        read(k + "type", type);
        read(k + "subtype", subtype);
        const auto type_value =
            vortex::utils::landmarks::landmark_type_from_string(type);
        const auto subtype_value =
            vortex::utils::landmarks::landmark_subtype_from_string(subtype);
        if (!type_value || !subtype_value) {
            throw std::runtime_error(fmt::format(
                "class '{}': type '{}' / subtype '{}': give the names of the "
                "vortex_msgs LandmarkType and LandmarkSubtype constants",
                name, type, subtype));
        }
        c.type = *type_value;
        c.subtype = *subtype_value;
        read(k + "symmetry_deg", c.symmetry_deg);
        read(k + "has_orientation", c.has_orientation);
        read(k + "prior", c.prior);
        read(k + "prior_radius_m", c.prior_radius_m);
        read(k + "group", c.group);
        if (c.group.empty()) {
            c.group = c.name;
        }
        int64_t max_instances = 0;
        read(k + "max_instances", max_instances);
        c.max_instances = static_cast<int>(max_instances);

        const std::string b = "markers.boxes." + name + ".";
        std::vector<double> size;
        read(b + "size", size);
        if (size.size() == 3) {
            MarkerBox box;
            std::copy_n(size.begin(), 3, box.size.begin());
            std::vector<double> v;
            read(b + "offset", v);
            if (v.size() == 3) {
                std::copy_n(v.begin(), 3, box.offset.begin());
            }
            v.clear();
            read(b + "color", v);
            if (v.size() == 3) {
                box.color = {v[0], v[1], v[2]};
            }
            read(b + "solid", box.solid);
            c.box = box;
        }
        cfg.classes.push_back(c);
    }
    cfg.validate();
    return cfg;
}

}  // namespace vortex::landmark_server

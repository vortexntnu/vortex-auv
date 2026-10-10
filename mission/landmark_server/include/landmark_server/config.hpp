#ifndef LANDMARK_SERVER__CONFIG_HPP_
#define LANDMARK_SERVER__CONFIG_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace vortex::landmark_server {

/// Value of the vortex_msgs constant with this name, e.g. "GATE".
std::optional<std::uint16_t> landmark_type(const std::string& name);
std::optional<std::uint16_t> landmark_subtype(const std::string& name);

/// One entry of `classes` in the config, see landmark_server.yaml.
struct ClassConfig {
    std::string name;
    std::uint16_t type{0};
    std::uint16_t subtype{0};
    double symmetry_deg{0.0};
    bool has_orientation{false};
    std::string prior;
    double prior_radius_m{5.0};
    int max_instances{0};
    std::string group;
};

struct GateParams {
    std::vector<std::string> panel_classes;
    double min_separation_m{0.2};
    double max_separation_m{2.5};
    double approach_m{1.0};
    double depth_below_panel_m{0.5};
};

struct SlalomParams {
    std::string red_class;
    std::string white_class;
    double nominal_spacing_m{1.52};
    double row_spacing_m{2.0};
    double min_spacing_m{0.8};
    double max_spacing_m{2.3};
};

struct TorpedoParams {
    std::string board_class;
    /// Opening name -> [y, z] from the board centre.
    std::map<std::string, std::array<double, 2>> openings;
};

/// Every value is explained in config/landmark_server.yaml.
struct Params {
    double keyframe_dist_m{0.5};
    double keyframe_time_s{1.0};
    int max_messages_per_keyframe{5};
    double max_range_m{10.0};

    double odom_sigma_trans_per_m{0.03};
    double odom_sigma_yaw_per_m{0.03};
    double odom_min_sigma_trans{0.01};
    double odom_min_sigma_yaw{0.002};
    double odom_sigma_roll_pitch{0.01};
    double odom_sigma_z{0.02};
    double attitude_sigma{0.02};
    double depth_sigma{0.05};

    double bearing_sigma{0.03};
    double range_sigma_a{0.1};
    double range_sigma_b{0.05};
    double orientation_yaw_sigma{0.1};
    double orientation_roll_pitch_sigma{0.5};
    double dcs_phi{1.0};
    int max_merged_per_factor{4};

    double gate_prob{0.999};
    double ambiguity_d2{4.6};

    double candidate_radius_m{0.5};
    int confirm_hits{5};
    double confirm_window_s{3.0};

    double merge_radius_m{0.5};

    GateParams gate;
    SlalomParams slalom;
    TorpedoParams torpedo;

    double range_sigma(double range) const {
        return range_sigma_a + range_sigma_b * range;
    }
};

struct Config {
    Params params;
    std::vector<ClassConfig> classes;

    const ClassConfig* find_class(std::uint16_t type,
                                  std::uint16_t subtype) const;
    const ClassConfig* find_class(const std::string& name) const;

    /// Throws std::runtime_error on a bad value.
    void validate() const;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__CONFIG_HPP_

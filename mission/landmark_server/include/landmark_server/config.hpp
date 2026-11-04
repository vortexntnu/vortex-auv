#ifndef LANDMARK_SERVER__CONFIG_HPP_
#define LANDMARK_SERVER__CONFIG_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vortex::landmark_server {

/// Real-size box drawn for a class in the debug markers.
struct MarkerBox {
    std::array<double, 3> size{};    // x out of the front, y right, z down
    std::array<double, 3> offset{};  // box centre from the landmark
    std::optional<std::array<double, 3>> color;
    bool solid{false};
};

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
    std::optional<MarkerBox> box;
};

struct GraphParams {
    double keyframe_dist_m{0.5};
    double keyframe_time_s{1.0};
    double sigma_trans_per_m{0.03};
    double sigma_yaw_per_m{0.03};
    double min_sigma_trans{0.01};
    double min_sigma_yaw{0.002};
    double sigma_roll_pitch{0.01};
    double sigma_z{0.02};
    double attitude_sigma{0.02};
    double depth_sigma{0.05};
    double bearing_sigma{0.03};
    double range_sigma_a{0.1};
    double range_sigma_b{0.05};
    double dcs_phi{1.0};
};

struct AssociationParams {
    double gate_prob{0.999};
    double ambiguity_d2{4.6};
    double candidate_radius_m{0.5};
    int confirm_hits{5};
    double confirm_window_s{3.0};
    double merge_radius_m{0.5};
};

/// Every value is explained in config/landmark_server.yaml.
struct Params {
    double max_range_m{10.0};
    AssociationParams association;
    GraphParams graph;

    // Measure defaults, used when the goal gives 0.
    double cluster_radius_m{0.5};
    double measure_duration_s{5.0};
    double measure_max_range_m{4.0};
    int measure_min_hits{5};
    double measure_max_spread_m{0.3};
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

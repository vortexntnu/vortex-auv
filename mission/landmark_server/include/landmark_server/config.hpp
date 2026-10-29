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

struct GateParams {
    std::vector<std::string> panel_classes;
    double min_separation_m{0.2};
    double max_separation_m{2.5};
    double approach_m{1.0};
    double depth_below_panel_m{0.5};
};

/// Every value is explained in config/landmark_server.yaml.
struct Params {
    double max_range_m{10.0};
    double cluster_radius_m{0.5};
    int confirm_hits{5};
    int max_hits_per_class{500};
    double max_hit_age_s{0.0};

    // Measure defaults, used when the goal gives 0.
    double measure_duration_s{5.0};
    double measure_max_range_m{4.0};
    int measure_min_hits{5};
    double measure_max_spread_m{0.3};

    GateParams gate;
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

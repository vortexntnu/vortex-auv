#ifndef LANDMARK_SERVER__CLASS_CONFIG_HPP_
#define LANDMARK_SERVER__CLASS_CONFIG_HPP_

#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <map>
#include <optional>
#include <pose_filtering/lib/typedefs.hpp>
#include <string>
#include <utility>
#include <vector>

namespace vortex::mission {

using vortex::filtering::LandmarkClassKey;

/// Axis-aligned box in the course frame [m] (x through the gate, y right).
struct LaneBox {
    double x_min{-1e9};
    double x_max{1e9};
    double y_min{-1e9};
    double y_max{1e9};

    bool contains(double x, double y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

/// Intake rules: pipe distance limit and the no-orientation variance.
struct IntakeConfig {
    /// Slalom pipes farther than this from the vehicle are discarded [m].
    double max_pipe_distance_m{7.0};
    /// A rotational covariance diagonal >= this means "no orientation".
    double no_orientation_rot_variance{1000.0};
    /// Extra measurement variance [m^2] = base + per_meter * distance from
    /// the vehicle. Far detections weigh less in the tracker.
    double noise_base_variance{0.0};
    double noise_variance_per_meter{0.0};
};

struct CourseFrameConfig {
    bool publish_tf{true};
    std::string frame_id{"nautilus/course"};
    /// Gate yaw estimates that must agree before the frame locks to the gate.
    int gate_lock_consistent_estimates{10};
    double gate_lock_max_yaw_std_deg{3.0};
    /// Start-vs-gate heading deviation that gives a warning [deg].
    double warn_start_vs_gate_deg{30.0};
    LaneBox before_gate{-10.0, 45.0, -12.0, 12.0};
    LaneBox after_gate{-5.0, 40.0, -6.0, 6.0};
};

/// Rules per landmark class (type + subtype).
struct ClassRule {
    int max_instances{20};
    /// A new track within this distance of a remembered landmark of the same
    /// class takes over its id [m]. <= 0 uses rules.plausibility_radius_m.
    double instance_gate_m{0.5};
    /// Remembered for the rest of the run.
    bool retain_forever{false};
    /// Otherwise forgotten this long after the last measurement [s].
    double retain_sec{15.0};
    /// Landmarks with at least this many observations are never forgotten
    /// (0 = off).
    int keep_after_observations{0};
    /// New landmarks closer than this to a large structure are rejected [m]
    /// (0 = off). Keeps gate legs from becoming slalom pipes.
    double min_distance_to_large_structures_m{0.0};
};

/// Offsets from a torpedo board icon to its opening, in the board frame
/// (x out of the front, y right, z down) [m].
struct TorpedoIconOffsets {
    Eigen::Vector3d fire{Eigen::Vector3d::Zero()};
    Eigen::Vector3d blood{Eigen::Vector3d::Zero()};
    Eigen::Vector3d firetruck{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ambulance{Eigen::Vector3d::Zero()};
};

/// Objects that lie on the pool floor or at the water surface get that depth,
/// instead of the (noisy) measured one. Depths are odom z (NED, down positive).
struct ZLockConfig {
    bool enable{false};
    double floor_z{0.0};
    double surface_z{0.0};
    /// (type, subtype); subtype 0 = every subtype of the type.
    std::vector<std::pair<uint16_t, uint16_t>> floor_classes;
    std::vector<std::pair<uint16_t, uint16_t>> surface_classes;

    bool is_floor(const LandmarkClassKey& key) const;
    bool is_surface(const LandmarkClassKey& key) const;
};

/// Rules that derive structure from parts.
struct MapRulesConfig {
    ZLockConfig z_lock;
    bool gate_yaw_from_panels{true};
    bool board_yaw_from_icons{true};
    bool bin_role_from_down_icons{true};
    bool octagon_from_table{true};
    bool torpedo_targets_from_icons{true};
    /// Yaw estimates that must agree before the yaw is locked.
    int yaw_lock_consistent_estimates{5};
    /// Estimates within this many degrees of the running mean agree.
    double yaw_agree_deg{10.0};
    /// Estimates that differ this much from the current yaw are ignored [deg].
    double yaw_max_jump_deg{90.0};
    /// Bin role icon and bin must be this close (xy) [m].
    double bin_role_radius_m{0.6};
    /// Icons of a pair must be this far apart (xy) to give a board yaw [m].
    double min_icon_separation_m{0.05};
    /// Panels must be this far apart (xy) to give a gate yaw [m].
    double min_panel_separation_m{0.3};
    /// Board offsets from icon to opening, per board version (1 and 2).
    TorpedoIconOffsets torpedo_version_1;
    TorpedoIconOffsets torpedo_version_2;
};

/// A box drawn for a class in the markers: its size in the landmark frame
/// (x out of the front, y right, z down) and the offset of its centre from
/// the landmark position, in the same frame. A solid box is the object
/// itself (a PVC pipe) and replaces the point; else it is a see-through
/// outline of a large structure around the point.
struct MarkerBox {
    Eigen::Vector3d size{Eigen::Vector3d::Zero()};
    Eigen::Vector3d offset{Eigen::Vector3d::Zero()};
    bool solid{false};
    /// RGB in [0, 1]; empty uses the colour of the type.
    std::optional<Eigen::Vector3d> color;
};

struct LandmarkMapConfig {
    IntakeConfig intake;
    MapRulesConfig map_rules;
    CourseFrameConfig course_frame;
    ClassRule default_rule;
    std::map<std::pair<uint16_t, uint16_t>, ClassRule> class_rules;
    /// A new track this close to a remembered landmark of a class with
    /// max_instances == 1 is the same object [m].
    double plausibility_radius_m{3.0};
    /// Classes that count as large structures, as (type, subtype); subtype 0
    /// stands for every subtype of the type.
    std::vector<std::pair<uint16_t, uint16_t>> large_structures;

    /// Boxes for the markers, per (type, subtype); subtype 0 stands for
    /// every subtype of the type.
    std::map<std::pair<uint16_t, uint16_t>, MarkerBox> marker_boxes;

    const ClassRule& rule_for(const LandmarkClassKey& key) const;
    bool is_large_structure(const LandmarkClassKey& key) const;
    /// The box for a class (the subtype entry wins over the type), or null.
    const MarkerBox* marker_box_for(const LandmarkClassKey& key) const;
};

/**
 * @brief Parse a class name: a type ("GATE") gives {type, 0}; a full subtype
 * constant ("BIN_STRUCTURE") gives {type, subtype}.
 */
std::optional<std::pair<uint16_t, uint16_t>> parse_class_name(
    const std::string& name);

/// Type name ("GATE") to LandmarkType value.
std::optional<uint16_t> parse_landmark_type(const std::string& name);

/// Subtype name to value, for the given type. Both the short ("WHITE") and
/// the full ("SLALOM_PIPE_WHITE") constant names are accepted.
std::optional<uint16_t> parse_landmark_subtype(uint16_t type,
                                               const std::string& name);

/// Readable name of a class: the subtype constant ("GATE_WHOLE") when known,
/// else the type name plus the subtype value ("GATE/9"), else "type/subtype".
std::string class_name(const LandmarkClassKey& key);

/// All subtype values known for a type (empty if unknown).
std::vector<uint16_t> known_subtypes(uint16_t type);

/**
 * @brief Parse the map configuration from a YAML tree with the keys `intake`,
 * `course_frame`, `classes`, `rules` and `markers` (see
 * config/landmark_server_config.yaml). Missing keys keep their defaults.
 * @throws std::runtime_error on unknown class or subtype names.
 */
LandmarkMapConfig parse_map_config(const YAML::Node& root);

/**
 * @brief Per-class track configs from the `track_config` tree: every key
 * other than `default` is a class name whose fields override @p default_config.
 * A class without subtype applies to all of its subtypes.
 * @return The (class key, config) pairs for TrackManagerConfig.
 */
std::vector<std::pair<LandmarkClassKey, vortex::filtering::LandmarkClassConfig>>
parse_per_class_track_config(
    const YAML::Node& track_config,
    const vortex::filtering::LandmarkClassConfig& default_config);

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__CLASS_CONFIG_HPP_

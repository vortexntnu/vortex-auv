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
#include "landmark_server/course_model.hpp"

namespace vortex::mission {

using vortex::filtering::LandmarkClassKey;

/// How far off a detection is, growing with the range d [m]: std along the
/// line of sight (depth) base + along_per_m * d, across it base +
/// across_per_m * d (a camera knows the direction much better than the
/// distance). One model, measured once (README, test C), used by the tracker
/// (on top of its sens_mod_std_dev) and by the graph. All zero: off.
struct DetectorNoise {
    double base_std_m{0.06};
    double along_std_per_m{0.03};
    double across_std_per_m{0.003};

    bool enabled() const {
        return base_std_m > 0.0 || along_std_per_m > 0.0 || across_std_per_m > 0.0;
    }
    /// Covariance of a detection at offset `ray` from the vehicle.
    Eigen::Matrix3d covariance(const Eigen::Vector3d& ray) const;
};

/// Intake: the detector noise and the no-orientation variance.
struct IntakeConfig {
    /// A rotational covariance diagonal >= this means "no orientation".
    double no_orientation_rot_variance{1000.0};
    /// Read at start (the graph is built with it).
    DetectorNoise noise;
    /// Use the position covariance of the detection (rotated into the target
    /// frame) instead of the class noise and the detector noise, when it has
    /// a positive diagonal. Scaled by covariance_scale (to test an over-
    /// or underconfident detector); each std at least covariance_min_std_m.
    bool use_measurement_covariance{false};
    double covariance_scale{1.0};
    double covariance_min_std_m{0.02};
};

struct CourseFrameConfig {
    bool publish_tf{true};
    std::string frame_id{"nautilus/course"};
    /// Gate yaw estimates that must agree before the frame locks to the gate.
    int gate_lock_consistent_estimates{10};
    double gate_lock_max_yaw_std_deg{3.0};
    /// Start-vs-gate heading deviation that gives a warning [deg].
    double warn_start_vs_gate_deg{30.0};
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
    /// Never a landmark, only tracked live. For things that are moved
    /// during the run (the items on the table): a remembered position would
    /// be wrong once they are moved.
    bool live_only{false};
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

/// Rules that derive structure from parts (map_rules.hpp).
struct MapRulesConfig {
    ZLockConfig z_lock;
    bool bin_role_from_down_icons{true};
    bool octagon_from_table{true};
    /// Bin role icon and bin must be this close (xy) [m].
    double bin_role_radius_m{0.6};
    /// Gate panels closer or farther apart than this (xy) are not one gate:
    /// no course frame estimate from them [m] (they hang ~1.6 m apart).
    double min_panel_separation_m{0.3};
    double max_panel_separation_m{2.5};
    /// Which of table and octagon gives the shared xy: "table" (the octagon
    /// is put over the table), "octagon" (the table under the octagon) or
    /// "midpoint" (both at the midpoint when both are measured). The one that
    /// is missing is derived from the other either way.
    std::string table_octagon_primary{"table"};
    /// Height of the table top above the floor [m]: a table derived from the
    /// octagon is put at floor_z - this (only with z_lock on).
    double table_height_m{0.7};
};

/// A box drawn for a class in the markers: its size in the landmark frame
/// (x out of the front, y right, z down) and the offset of its centre from
/// the landmark position, in the same frame. A solid box is the object
/// itself (a PVC pipe); else it is a see-through outline of the prop around
/// the point. Display only.
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
    /// A remembered landmark takes a new track over only when it is clearly
    /// the nearest: the next landmark of the class must be at least this many
    /// times farther away. <= 1 turns the check off.
    double adoption_ambiguity_ratio{2.0};
    /// How long an ambiguous track waits for the ambiguity to resolve before
    /// it is treated as a new object (new landmark, or rejected when the
    /// class is full) [s].
    double adoption_wait_sec{2.0};

    /// Boxes for the markers, per (type, subtype); subtype 0 stands for
    /// every subtype of the type.
    std::map<std::pair<uint16_t, uint16_t>, MarkerBox> marker_boxes;

    /// The course layout: tasks, their templates and prior poses (config
    /// `course`). Read at start.
    CourseConfig course;

    const ClassRule& rule_for(const LandmarkClassKey& key) const;
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
 * `detector_noise`, `course_frame`, `classes`, `rules`, `markers` and
 * `course` (the config files). Missing keys keep their defaults.
 * @throws std::runtime_error on unknown class or subtype names, unknown keys
 * in the course, and keys that were removed (with what replaced them).
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

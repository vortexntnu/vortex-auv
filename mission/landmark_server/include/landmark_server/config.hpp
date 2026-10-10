#ifndef LANDMARK_SERVER__CONFIG_HPP_
#define LANDMARK_SERVER__CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace vortex::landmark_server {

/// One object class (config `classes.<name>`).
struct ClassConfig {
    std::string name;
    /// LandmarkType / LandmarkSubtype values the detectors publish.
    std::uint16_t type{0};
    std::uint16_t subtype{0};
    /// 0 = no symmetry; 90, 180: yaw only known modulo this; 360: yaw
    /// meaningless.
    double symmetry_deg{0.0};
    /// Use the detector's orientation when it sends one.
    bool has_orientation{false};
    /// Prior map label of the task the class belongs to ("" = none): a new
    /// landmark of the class is only made within prior_radius_m (xy) of it.
    std::string prior;
    double prior_radius_m{5.0};
    /// Landmarks of the class shown at most (the most observed); 0 = any.
    int max_instances{0};
};

/// Gate frames from the two role panels.
struct GateParams {
    std::vector<std::string> panel_classes;  // two, or none = off
    double min_separation_m{0.2};
    double max_separation_m{2.5};
    double approach_m{1.0};
    double depth_below_panel_m{0.5};
};

/// Everything tunable (config/landmark_server.yaml explains each value).
struct Params {
    // Keyframes and intake.
    double keyframe_dist_m{0.5};
    double keyframe_time_s{1.0};
    int max_messages_per_keyframe{5};
    double max_range_m{10.0};

    // Odometry noise per keyframe step.
    double odom_sigma_trans_per_m{0.03};
    double odom_sigma_yaw_per_m{0.03};
    double odom_min_sigma_trans{0.01};
    double odom_min_sigma_yaw{0.002};
    double odom_sigma_roll_pitch{0.01};
    double odom_sigma_z{0.02};
    // Absolute measurements per keyframe (IMU gravity, pressure).
    double attitude_sigma{0.02};
    double depth_sigma{0.05};

    // Detection noise.
    double bearing_sigma{0.03};
    double range_sigma_a{0.1};
    double range_sigma_b{0.05};
    double orientation_yaw_sigma{0.1};
    double orientation_roll_pitch_sigma{0.5};
    double dcs_phi{1.0};
    int max_merged_per_factor{4};

    // Association.
    double gate_prob{0.999};
    double ambiguity_d2{4.6};

    // New landmarks.
    double candidate_radius_m{0.5};
    int confirm_hits{5};
    double confirm_window_s{3.0};

    // Map upkeep.
    double merge_radius_m{0.5};

    GateParams gate;

    /// Range noise sigma_r = a + b * r.
    double range_sigma(double range) const {
        return range_sigma_a + range_sigma_b * range;
    }
};

struct Config {
    Params params;
    std::vector<ClassConfig> classes;

    /// The class of a detection, or nullptr if it is not configured.
    const ClassConfig* find_class(std::uint16_t type,
                                  std::uint16_t subtype) const;
    const ClassConfig* find_class(const std::string& name) const;

    /// Throws std::runtime_error with the reason on a bad value.
    void validate() const;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__CONFIG_HPP_

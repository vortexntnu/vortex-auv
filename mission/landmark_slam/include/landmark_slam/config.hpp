#ifndef LANDMARK_SLAM__CONFIG_HPP_
#define LANDMARK_SLAM__CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vortex::landmark_slam {

/// One object class (landmark_classes.yaml).
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
};

/// A landmark from prior_map.yaml (map frame): where an object of its class
/// may be. Votes for a new landmark of the class are only taken within
/// 3 sigma_xy + vote_radius_m of one of its entries.
struct PriorLandmark {
    std::string class_name;
    double x{0.0}, y{0.0};
    double sigma_xy{0.0};
};

/// Start pose of the vehicle in the map frame (prior_map.yaml).
struct InitialPose {
    double x{0.0}, y{0.0}, yaw{0.0};
    /// Unset: the pressure depth (odom z) is the map z.
    std::optional<double> z;
    double sigma_xy{0.5}, sigma_yaw{0.3};
};

/// Gate frames from the two role panels (gate_middle, and per panel
/// <panel>_entrance / <panel>_exit through its opening).
struct GateParams {
    std::vector<std::string> panel_classes;  // two, or none = off
    double min_separation_m{0.2};
    double max_separation_m{2.5};
    /// Entrance / exit this far before / after the gate line.
    double approach_m{1.0};
    /// The opening is this far below the panel (z down).
    double depth_below_panel_m{0.5};
};

/// ROS parameters (params.yaml).
struct Params {
    bool use_prior_map{true};
    std::string prior_map_file;
    std::string classes_file;
    double keyframe_dist_m{0.5};
    double keyframe_time_s{1.0};
    double odom_sigma_trans_per_m{0.03};
    double odom_sigma_yaw_per_m{0.01};
    double default_prior_sigma_xy{1.0};
    double gate_prob{0.95};
    double min_votes{3.0};
    double vote_radius_m{0.5};
    double bearing_sigma{0.03};
    double range_sigma_a{0.1};
    double range_sigma_b{0.05};
    GateParams gate;

    /// Range noise sigma_r = a + b * r.
    double range_sigma(double range) const {
        return range_sigma_a + range_sigma_b * range;
    }
};

struct Config {
    Params params;
    std::vector<ClassConfig> classes;
    /// Set when a prior map is used: the start pose in the map frame.
    /// Without it the map frame is odom at startup.
    std::optional<InitialPose> initial_pose;
    std::vector<PriorLandmark> prior_landmarks;

    /// The class of a detection, or nullptr if it is not configured.
    const ClassConfig* find_class(std::uint16_t type,
                                  std::uint16_t subtype) const;
};

/**
 * @brief Read the class file and (if use_prior_map) the prior map named in
 * params. Throws std::runtime_error with the reason on a bad file or an
 * unknown class.
 */
Config load_config(const Params& params);

}  // namespace vortex::landmark_slam

#endif  // LANDMARK_SLAM__CONFIG_HPP_

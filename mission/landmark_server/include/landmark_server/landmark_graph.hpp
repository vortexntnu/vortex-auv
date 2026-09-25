#ifndef LANDMARK_SERVER__LANDMARK_GRAPH_HPP_
#define LANDMARK_SERVER__LANDMARK_GRAPH_HPP_

#include <yaml-cpp/yaml.h>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <vector>

namespace vortex::mission {

/**
 * @brief Settings of the smoothing backend (config `graph`).
 */
struct LandmarkGraphConfig {
    bool enable{false};

    /// A new keyframe when the vehicle has moved or turned this much since
    /// the last one, or after this long (0 = never by time).
    double keyframe_distance_m{0.5};
    double keyframe_angle_deg{10.0};
    double keyframe_interval_sec{5.0};

    /// Odometry noise between two keyframes (std of one keyframe step),
    /// growing with the distance travelled (and, for the yaw, the time: gyro
    /// bias). The steps are independent in the graph, so a drift that is
    /// really a bias (the same every metre) needs a larger value than the
    /// drift per metre itself.
    double odom_pos_std_per_m{0.02};
    double odom_yaw_std_deg_per_m{1.0};
    double odom_yaw_std_deg_per_sec{0.01};
    double odom_min_pos_std_m{0.01};
    double odom_min_rot_std_deg{0.1};

    /// Roll, pitch (IMU) and depth (pressure) do not drift: every keyframe
    /// gets them from odometry as an absolute measurement.
    double roll_pitch_std_deg{1.0};
    double depth_std_m{0.05};

    /// Huber threshold of the landmark measurements, in standard deviations
    /// (a wrong association should not pull the whole map).
    double huber_k{2.0};
    /// Landmark measurements per landmark and keyframe; more are dropped
    /// (a hovering vehicle sees the same thing many times a second).
    int max_measurements_per_keyframe{3};
    /// The smoothed position is used once a landmark has this many
    /// measurements in the graph.
    int min_observations{3};
    /// Measurements of a track that is not in the map yet wait for it (per
    /// track, oldest dropped first).
    int max_pending_per_track{50};
};

/// Read `graph` (the node under the key). Missing keys keep the defaults.
LandmarkGraphConfig parse_graph_config(const YAML::Node& node);

/**
 * @brief Landmark SLAM backend: an iSAM2 factor graph of vehicle keyframes
 * and landmark positions. ROS-free.
 *
 * Odometry drifts in x, y and yaw. The graph links keyframes by the odometry
 * between them (noise growing with the distance), anchors roll, pitch and
 * depth absolutely, and ties each keyframe to the landmarks it saw. When a
 * landmark is seen again after the odometry has drifted, the graph moves
 * the keyframes, and with them every landmark they saw, to agree.
 *
 * Frames: "odom" is the raw odometry frame, the one the controller uses; the
 * graph frame is odom as it was at the first keyframe. The results are given
 * in the current odom frame (landmark_in_odom()): a landmark is placed where
 * it is relative to the vehicle according to the graph, then expressed with
 * the vehicle's raw odometry pose. The vehicle can then steer on odometry
 * without knowing about the graph.
 *
 * Measurements are landmark positions already in the odom frame (at their
 * own stamp). With the raw odometry pose of the nearest keyframe they become
 * positions relative to that keyframe; the short piece of odometry between
 * the two is taken as exact.
 */
class LandmarkGraph {
   public:
    explicit LandmarkGraph(LandmarkGraphConfig config);
    ~LandmarkGraph();
    LandmarkGraph(const LandmarkGraph&) = delete;
    LandmarkGraph& operator=(const LandmarkGraph&) = delete;

    const LandmarkGraphConfig& config() const { return config_; }

    /**
     * @brief Raw odometry: the vehicle pose in the odom frame. Makes a
     * keyframe when the vehicle has moved or turned enough. Older stamps than
     * the last keyframe are ignored.
     */
    void add_odometry(double stamp, const Eigen::Isometry3d& odom_T_body);

    /**
     * @brief A landmark measurement.
     * @param landmark_id Stable id (the map id). A new id adds a landmark.
     * @param stamp Measurement time [s].
     * @param position Position in the odom frame at that time.
     * @param covariance Position covariance [m^2] in the odom frame.
     * @return False if dropped (no keyframe yet, or the keyframe has enough
     * measurements of this landmark).
     */
    bool add_measurement(int landmark_id,
                         double stamp,
                         const Eigen::Vector3d& position,
                         const Eigen::Matrix3d& covariance);

    /// One iSAM2 update with what was added since the last one.
    void optimize();

    /**
     * @brief Smoothed landmark position in the current odom frame, once the
     * landmark has min_observations measurements.
     */
    std::optional<Eigen::Vector3d> landmark_in_odom(int landmark_id) const;

    /// odom_T_graph: the graph frame expressed in the current odom frame
    /// (identity until odometry has drifted and been corrected).
    Eigen::Isometry3d correction() const;

    /// Smoothed vehicle pose of the newest keyframe, graph frame.
    std::optional<Eigen::Isometry3d> latest_keyframe_estimate() const;

    /// Every keyframe as the graph sees it now, in the current odom frame
    /// (for display: the smoothed trajectory).
    std::vector<Eigen::Isometry3d> keyframes_in_odom() const;

    /// Every keyframe as the raw odometry gave it.
    std::vector<Eigen::Isometry3d> keyframes_raw() const;

    /// Measurements of a landmark in the graph (0 if unknown).
    int observations(int landmark_id) const;

    std::size_t keyframe_count() const;
    std::size_t landmark_count() const;

    /// Forget everything; the next odometry starts a new graph.
    void clear();

   private:
    struct Impl;
    LandmarkGraphConfig config_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_GRAPH_HPP_

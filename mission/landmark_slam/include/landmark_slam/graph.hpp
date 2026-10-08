#ifndef LANDMARK_SLAM__GRAPH_HPP_
#define LANDMARK_SLAM__GRAPH_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "landmark_slam/config.hpp"

namespace vortex::landmark_slam {

inline gtsam::Key X(int kf) {
    return gtsam::Symbol('x', static_cast<std::uint64_t>(kf));
}
inline gtsam::Key L(int id) {
    return gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

using BearingRange = gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>;

/// A detection in the base frame of the keyframe it is attached to.
struct Measurement {
    gtsam::Point3 position;
    /// Object orientation relative to the base, when the detector gives one.
    std::optional<gtsam::Rot3> rotation;
};

/// What the map knows about one landmark.
struct LandmarkState {
    int id{0};
    ClassConfig cls;
    gtsam::Pose3 pose;
    /// Marginal covariance in the landmark's tangent space (GTSAM order:
    /// rotation, then translation, both in the landmark frame).
    gtsam::Matrix6 cov{gtsam::Matrix6::Identity()};
    /// Position covariance relative to the vehicle (latest keyframe), map
    /// axes: what navigating to it depends on. Unlike cov, it leaves out
    /// how well the map frame itself is known (the prior map's start pose).
    gtsam::Matrix3 relative_cov{gtsam::Matrix3::Identity()};
    int n_obs{0};
    double first_seen{0.0};
    double last_seen{0.0};
    /// Yaw is meaningful: from an orientation measurement.
    bool yaw_known{false};
};

/// Bearing-range noise sigmas [bearing, bearing, range] at this range.
gtsam::Vector3 bearing_range_sigmas(const Params& p, double range);

/**
 * @brief iSAM2 graph of vehicle keyframes X(k) and landmarks L(id), both
 * Pose3 in the map frame.
 */
class LandmarkGraph {
   public:
    /// Clear everything; X(0) at the start pose (prior map) or odom.
    void reset(const Config& cfg, const gtsam::Pose3& T_odom_base, double t);
    /// New keyframe: odometry between factor, depth and roll/pitch.
    int add_keyframe(const gtsam::Pose3& T_odom_base, double t);
    /// New landmark at init.
    void add_landmark(int id,
                      const ClassConfig& cls,
                      const gtsam::Pose3& init,
                      double t);
    void add_observation(int kf, int id, const Measurement& m, double t);
    /// isam.update() with what was added, refresh the estimates.
    void update();
    /// Refresh the landmark covariances (once per keyframe, before
    /// publishing).
    void update_covariances();

    int last_keyframe() const {
        return static_cast<int>(keyframes_.size()) - 1;
    }
    double keyframe_time(int kf) const { return keyframes_.at(kf).t; }
    const gtsam::Pose3& keyframe_odom(int kf) const {
        return keyframes_.at(kf).T_odom_base;
    }
    gtsam::Pose3 keyframe_pose(int kf) const;
    gtsam::Pose3 landmark_pose(int id) const;
    bool has_landmark(int id) const { return landmarks_.count(id) > 0; }
    /// map -> odom from the latest keyframe.
    gtsam::Pose3 map_to_odom() const;

    /// Joint marginal covariance (includes the cross-covariances).
    gtsam::JointMarginal joint_cov(const gtsam::KeyVector& keys) const;
    const std::vector<LandmarkState>& landmarks() const { return cache_; }
    const Config& config() const { return cfg_; }

   private:
    struct Keyframe {
        gtsam::Pose3 T_odom_base;
        double t{0.0};
    };

    void add_absolute_factors(int kf, const gtsam::Pose3& T_odom_base);

    Config cfg_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
    gtsam::Values estimate_;
    std::vector<Keyframe> keyframes_;
    std::map<int, LandmarkState> landmarks_;
    std::vector<LandmarkState> cache_;
    /// Depth offset: map z = odom z + z_offset_.
    double z_offset_{0.0};
};

}  // namespace vortex::landmark_slam

#endif  // LANDMARK_SLAM__GRAPH_HPP_
